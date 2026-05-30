param(
    [string]$TargetRoot = 'E:\codex',
    [switch]$ForceStop,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$logDir = Join-Path $TargetRoot 'migration_logs'
$backupRoot = Join-Path $TargetRoot ("original_location_backups_{0}" -f $stamp)

function Write-Step {
    param([string]$Message)
    Write-Host ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $Message)
}

function Test-ReparsePoint {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        return $false
    }
    $item = Get-Item -LiteralPath $Path -Force
    return [bool]($item.Attributes -band [IO.FileAttributes]::ReparsePoint)
}

function Get-SafePathName {
    param([string]$Path)
    return ($Path -replace '^[A-Za-z]:\\?', '' -replace '[:\\\/\s]+', '_')
}

function Invoke-MirrorCopy {
    param(
        [string]$Source,
        [string]$Target,
        [string]$Name
    )

    if (-not (Test-Path -LiteralPath $Source)) {
        Write-Step "skip missing: $Source"
        return
    }

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Target) | Out-Null
    $log = Join-Path $logDir ("robocopy_{0}_{1}.log" -f $Name, $stamp)
    Write-Step "mirror $Source -> $Target"
    if ($DryRun) {
        return
    }

    & robocopy $Source $Target /MIR /XJ /FFT /R:2 /W:2 /NP /LOG:$log | Out-Null
    $code = $LASTEXITCODE
    if ($code -gt 7) {
        throw "robocopy failed for $Source -> $Target, exit code $code, log: $log"
    }
}

function Switch-ToJunction {
    param(
        [string]$Source,
        [string]$Target,
        [string]$Name
    )

    if (-not (Test-Path -LiteralPath $Source)) {
        Write-Step "skip missing source for junction: $Source"
        return
    }

    if (Test-ReparsePoint $Source) {
        Write-Step "already a junction/reparse point: $Source"
        return
    }

    Invoke-MirrorCopy -Source $Source -Target $Target -Name $Name

    $parent = Split-Path -Parent $Source
    $leaf = Split-Path -Leaf $Source
    $tempName = "{0}.before_e_migration_{1}" -f $leaf, $stamp
    $tempPath = Join-Path $parent $tempName
    $backupName = "{0}_{1}" -f (Get-SafePathName $Source), $stamp
    $backupPath = Join-Path $backupRoot $backupName

    Write-Step "replace $Source with junction to $Target"
    if ($DryRun) {
        return
    }

    Rename-Item -LiteralPath $Source -NewName $tempName
    New-Item -ItemType Junction -Path $Source -Target $Target | Out-Null

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $backupPath) | Out-Null
    Move-Item -LiteralPath $tempPath -Destination $backupPath
    Write-Step "moved original backup to $backupPath"
}

function Move-SameDriveToJunction {
    param(
        [string]$Source,
        [string]$Target,
        [string]$Name
    )

    if (-not (Test-Path -LiteralPath $Source)) {
        Write-Step "skip missing same-drive source: $Source"
        return
    }

    if (Test-ReparsePoint $Source) {
        Write-Step "already a junction/reparse point: $Source"
        return
    }

    Write-Step "move $Source -> $Target and leave compatibility junction"
    if ($DryRun) {
        return
    }

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Target) | Out-Null
    if (Test-Path -LiteralPath $Target) {
        Invoke-MirrorCopy -Source $Source -Target $Target -Name $Name
        $backupName = "{0}_{1}" -f (Get-SafePathName $Source), $stamp
        $backupPath = Join-Path $backupRoot $backupName
        Move-Item -LiteralPath $Source -Destination $backupPath
    } else {
        Move-Item -LiteralPath $Source -Destination $Target
    }
    New-Item -ItemType Junction -Path $Source -Target $Target | Out-Null
}

function Get-BlockingProcesses {
    $patterns = @(
        [regex]::Escape("$env:USERPROFILE\.codex"),
        [regex]::Escape("$env:USERPROFILE\.vscode\extensions\openai.chatgpt"),
        [regex]::Escape("$env:USERPROFILE\.vscode\extensions\hiztam.codex-history-viewer"),
        [regex]::Escape("$env:APPDATA\Code\User\globalStorage\hiztam.codex-history-viewer"),
        [regex]::Escape("$env:LOCALAPPDATA\OpenAI"),
        [regex]::Escape("$env:LOCALAPPDATA\Packages\OpenAI.Codex_2p2nqsd0c76g0"),
        [regex]::Escape("$env:LOCALAPPDATA\Packages\OpenAI.ChatGPT-Desktop_2p2nqsd0c76g0"),
        [regex]::Escape('E:\codex_mcp_tools')
    )
    $pattern = ($patterns -join '|')

    Get-CimInstance Win32_Process |
        Where-Object {
            ($_.ExecutablePath -and $_.ExecutablePath -match $pattern) -or
            ($_.CommandLine -and $_.CommandLine -match $pattern) -or
            ($_.Name -in @('Codex.exe', 'codex.exe') -and $_.ExecutablePath -like 'C:\Program Files\WindowsApps\OpenAI.Codex*')
        } |
        Select-Object ProcessId, Name, ExecutablePath, CommandLine
}

New-Item -ItemType Directory -Force -Path $logDir | Out-Null
New-Item -ItemType Directory -Force -Path $TargetRoot | Out-Null

$transcript = Join-Path $logDir ("switch_to_e_{0}.log" -f $stamp)
Start-Transcript -Path $transcript | Out-Null
try {
    Write-Step "target root: $TargetRoot"

    $blocking = @(Get-BlockingProcesses)
    if ($blocking.Count -gt 0) {
        $procList = $blocking | ForEach-Object { "{0} PID={1} {2}" -f $_.Name, $_.ProcessId, $_.ExecutablePath }
        if ($ForceStop) {
            Write-Step "stopping blocking Codex/OpenAI processes"
            $blocking | ForEach-Object {
                Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
            }
            Start-Sleep -Seconds 2
        } else {
            Write-Host ""
            Write-Host "The following Codex/OpenAI processes are still using the source files:"
            $procList | ForEach-Object { Write-Host "  $_" }
            throw "Close Codex, VS Code Codex extension, and related MCP processes first, or rerun with -ForceStop."
        }
    }

    $items = @(
        @{Name='home_codex'; Source=(Join-Path $env:USERPROFILE '.codex'); Target=(Join-Path $TargetRoot 'home\.codex')},
        @{Name='home_agents'; Source=(Join-Path $env:USERPROFILE '.agents'); Target=(Join-Path $TargetRoot 'home\.agents')},
        @{Name='cache_codex_runtimes'; Source=(Join-Path $env:USERPROFILE '.cache\codex-runtimes'); Target=(Join-Path $TargetRoot 'home\.cache\codex-runtimes')},
        @{Name='appdata_local_openai'; Source=(Join-Path $env:LOCALAPPDATA 'OpenAI'); Target=(Join-Path $TargetRoot 'AppData\Local\OpenAI')},
        @{Name='package_openai_codex'; Source=(Join-Path $env:LOCALAPPDATA 'Packages\OpenAI.Codex_2p2nqsd0c76g0'); Target=(Join-Path $TargetRoot 'AppData\Local\Packages\OpenAI.Codex_2p2nqsd0c76g0')},
        @{Name='package_openai_chatgpt_desktop'; Source=(Join-Path $env:LOCALAPPDATA 'Packages\OpenAI.ChatGPT-Desktop_2p2nqsd0c76g0'); Target=(Join-Path $TargetRoot 'AppData\Local\Packages\OpenAI.ChatGPT-Desktop_2p2nqsd0c76g0')}
    )

    $extensionsRoot = Join-Path $env:USERPROFILE '.vscode\extensions'
    if (Test-Path -LiteralPath $extensionsRoot) {
        Get-ChildItem -LiteralPath $extensionsRoot -Directory -Force |
            Where-Object { $_.Name -like 'openai.chatgpt-*' -or $_.Name -like '*codex*' -or $_.Name -like '*chatgpt*' } |
            ForEach-Object {
                $items += @{
                    Name = 'vscode_extension_' + ($_.Name -replace '[^A-Za-z0-9_.-]', '_')
                    Source = $_.FullName
                    Target = Join-Path $TargetRoot ('vscode\extensions\' + $_.Name)
                }
            }
    }

    $globalStorageRoots = @(
        Join-Path $env:APPDATA 'Code\User\globalStorage',
        Join-Path $env:APPDATA 'Cursor\User\globalStorage'
    )
    foreach ($globalStorageRoot in $globalStorageRoots) {
        if (Test-Path -LiteralPath $globalStorageRoot) {
            Get-ChildItem -LiteralPath $globalStorageRoot -Directory -Force |
                Where-Object { $_.Name -match 'openai|codex|chatgpt' } |
                ForEach-Object {
                    $relativeRoot = $globalStorageRoot.Substring($env:APPDATA.Length).TrimStart('\')
                    $items += @{
                        Name = 'global_storage_' + ($_.Name -replace '[^A-Za-z0-9_.-]', '_')
                        Source = $_.FullName
                        Target = Join-Path $TargetRoot ('AppData\Roaming\' + $relativeRoot + '\' + $_.Name)
                    }
                }
        }
    }

    foreach ($item in $items) {
        Switch-ToJunction -Source $item.Source -Target $item.Target -Name $item.Name
    }

    Move-SameDriveToJunction -Source 'E:\codex_mcp_tools' -Target (Join-Path $TargetRoot 'codex_mcp_tools') -Name 'codex_mcp_tools'
    Move-SameDriveToJunction -Source 'E:\test_nohup_codex' -Target (Join-Path $TargetRoot 'misc\test_nohup_codex') -Name 'test_nohup_codex'

    [Environment]::SetEnvironmentVariable('CODEX_HOME', (Join-Path $TargetRoot 'home\.codex'), 'User')

    $manifest = Join-Path $TargetRoot 'CODEX_E_DRIVE_MIGRATION_RESULT.txt'
    @(
        "Codex E-drive migration completed at $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
        "TargetRoot=$TargetRoot"
        "CODEX_HOME=$(Join-Path $TargetRoot 'home\.codex')"
        "Original-location backups=$backupRoot"
        "Transcript=$transcript"
        ""
        "C-drive source paths are now directory junctions pointing into E:\codex."
        "The Windows Store application package under C:\Program Files\WindowsApps is system-managed and is not moved by this script."
    ) | Set-Content -Path $manifest -Encoding UTF8

    Write-Step "completed"
    Write-Step "manifest: $manifest"
} finally {
    Stop-Transcript | Out-Null
}
