[CmdletBinding()]
param(
    [string]$Root = "",
    [string]$OfficialSourceUrl = "https://github.com/RealManRobot/ros2_rm_robot/archive/refs/heads/humble.zip",
    [string]$ArchivePath = "",
    [string]$ExtractDir = "",
    [string]$WslDistro = "",
    [ValidateSet("VMware", "WSL", "Auto", "None")]
    [string]$RosCheckMode = "VMware",
    [string]$VmWorkspacePath = "~/rm65b_dual_arm_ws",
    [switch]$SkipDownload,
    [switch]$SkipWslCheck,
    [switch]$SkipProjectVerify,
    [switch]$ForceRefreshTarget
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Write-Status {
    param(
        [ValidateSet("OK", "INFO", "WARN", "FAIL")]
        [string]$Kind,
        [string]$Message
    )

    $color = switch ($Kind) {
        "OK" { "Green" }
        "INFO" { "Cyan" }
        "WARN" { "Yellow" }
        "FAIL" { "Red" }
    }
    Write-Host ("[{0}] {1}" -f $Kind, $Message) -ForegroundColor $color
}

function Join-ProjectPath {
    param(
        [string]$Base,
        [string]$Relative
    )

    $normalized = $Relative -replace "/", [IO.Path]::DirectorySeparatorChar
    return Join-Path $Base $normalized
}

function Get-MissingPaths {
    param(
        [string]$Base,
        [string[]]$RelativePaths
    )

    $missing = @()
    foreach ($relative in $RelativePaths) {
        $path = Join-ProjectPath $Base $relative
        if (!(Test-Path -LiteralPath $path)) {
            $missing += $relative
        }
    }
    return $missing
}

function Test-RequiredTree {
    param(
        [string]$Base,
        [string[]]$RelativePaths
    )

    if (!(Test-Path -LiteralPath $Base -PathType Container)) {
        return $false
    }
    return (@(Get-MissingPaths -Base $Base -RelativePaths $RelativePaths).Count -eq 0)
}

function Find-OfficialSource {
    param(
        [string[]]$Candidates,
        [string[]]$RequiredFiles
    )

    foreach ($candidate in $Candidates) {
        if (Test-RequiredTree -Base $candidate -RelativePaths $RequiredFiles) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    return $null
}

function Ensure-Directory {
    param([string]$Path)

    if (!(Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path | Out-Null
    }
}

function Invoke-CheckedCommand {
    param(
        [string]$FilePath,
        [string[]]$Arguments,
        [string]$FailureMessage
    )

    $output = & $FilePath @Arguments 2>&1
    $exit = $LASTEXITCODE
    if ($exit -ne 0) {
        Write-Host $output
        throw $FailureMessage
    }
    return $output
}

function Test-CommandAvailable {
    param([string]$Name)

    return $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")).Path
}
else {
    $Root = (Resolve-Path -LiteralPath $Root).Path
}
$downloadsDir = Join-Path $Root "downloads"
$workspace = Join-Path $Root "rm65b_dual_arm_ws"
$workspaceSrc = Join-Path $workspace "src"
$target = Join-Path $workspaceSrc "ros2_rm_robot"

if ([string]::IsNullOrWhiteSpace($ArchivePath)) {
    $ArchivePath = Join-Path $downloadsDir "ros2_rm_robot_humble.zip"
}
if ([string]::IsNullOrWhiteSpace($ExtractDir)) {
    $ExtractDir = Join-Path $downloadsDir "ros2_rm_robot_humble"
}

$officialRequiredFiles = @(
    "README.md",
    "rm_bringup/package.xml",
    "rm_description/package.xml",
    "rm_driver/package.xml",
    "rm_moveit2_config/rm_65_config/package.xml",
    "rm_ros_interfaces/package.xml"
)

$workspaceRequiredFiles = @(
    "scripts/verify_project.py",
    "src/rm65b_dual_arm_bringup/launch/full_system.launch.py",
    "src/rm65b_dual_arm_bringup/config/dual_arm_frames.yaml",
    "src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.urdf",
    "src/rm65b_dual_arm_moveit_config/launch/move_group.launch.py",
    "src/rm65b_dual_arm_planning/package.xml",
    "src/rm65b_gripper_control/package.xml",
    "src/rm65b_safety/package.xml",
    "src/rm65b_vision_guidance/package.xml",
    "src/rm65b_weaving_primitives/config/weaving_tree.xml"
)

$officialCandidates = @(
    (Join-Path $ExtractDir "ros2_rm_robot-humble"),
    (Join-Path $ExtractDir "ros2_rm_robot"),
    (Join-Path $downloadsDir "ros2_rm_robot-humble"),
    (Join-Path $downloadsDir "ros2_rm_robot")
)

Write-Status "INFO" "Root: $Root"
Write-Status "INFO" "Official source URL: $OfficialSourceUrl"

Ensure-Directory -Path $downloadsDir
Ensure-Directory -Path $workspaceSrc

$officialSource = Find-OfficialSource -Candidates $officialCandidates -RequiredFiles $officialRequiredFiles
if ($null -eq $officialSource) {
    if (!(Test-Path -LiteralPath $ArchivePath -PathType Leaf)) {
        if ($SkipDownload) {
            throw "Official source archive is missing and -SkipDownload was specified: $ArchivePath"
        }

        Write-Status "INFO" "Downloading official RealMan ROS2 Humble source archive."
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        $partialArchive = "$ArchivePath.partial"
        if (Test-Path -LiteralPath $partialArchive) {
            Remove-Item -LiteralPath $partialArchive -Force
        }
        Invoke-WebRequest -Uri $OfficialSourceUrl -OutFile $partialArchive -UseBasicParsing
        Move-Item -LiteralPath $partialArchive -Destination $ArchivePath -Force
        Write-Status "OK" "Downloaded: $ArchivePath"
    }
    else {
        Write-Status "OK" "Archive already exists: $ArchivePath"
    }

    Write-Status "INFO" "Extracting official source archive."
    Ensure-Directory -Path $ExtractDir
    Expand-Archive -LiteralPath $ArchivePath -DestinationPath $ExtractDir -Force
    $officialSource = Find-OfficialSource -Candidates $officialCandidates -RequiredFiles $officialRequiredFiles
}

if ($null -eq $officialSource) {
    throw "Could not find a valid official ros2_rm_robot source tree after extraction."
}
Write-Status "OK" "Official source tree: $officialSource"

if (Test-Path -LiteralPath $target) {
    if (Test-RequiredTree -Base $target -RelativePaths $officialRequiredFiles) {
        Write-Status "OK" "Workspace official source already exists: $target"
    }
    elseif ($ForceRefreshTarget) {
        Write-Status "WARN" "Refreshing invalid workspace official source: $target"
        Remove-Item -LiteralPath $target -Recurse -Force
        Copy-Item -LiteralPath $officialSource -Destination $target -Recurse -Force
        Write-Status "OK" "Copied official source to workspace: $target"
    }
    else {
        $missing = Get-MissingPaths -Base $target -RelativePaths $officialRequiredFiles
        throw "Workspace official source exists but is incomplete: $target`nMissing:`n  $($missing -join "`n  ")`nRe-run with -ForceRefreshTarget to replace it."
    }
}
else {
    Copy-Item -LiteralPath $officialSource -Destination $target -Recurse -Force
    Write-Status "OK" "Copied official source to workspace: $target"
}

$targetMissing = @(Get-MissingPaths -Base $target -RelativePaths $officialRequiredFiles)
if ($targetMissing.Count -gt 0) {
    throw "Official source verification failed under workspace:`n  $($targetMissing -join "`n  ")"
}
Write-Status "OK" "Official source verification passed."

$workspaceMissing = @(Get-MissingPaths -Base $workspace -RelativePaths $workspaceRequiredFiles)
if ($workspaceMissing.Count -gt 0) {
    throw "Workspace verification failed. Missing required files:`n  $($workspaceMissing -join "`n  ")"
}
Write-Status "OK" "Workspace required files are present."

if (!(Test-CommandAvailable -Name "Expand-Archive")) {
    throw "PowerShell Expand-Archive is not available."
}
Write-Status "OK" "PowerShell archive support is available."

$vmCheckScript = Join-Path $workspace "scripts\bootstrap_vmware_ubuntu.sh"
if (Test-Path -LiteralPath $vmCheckScript) {
    Write-Status "OK" "Ubuntu/VMware environment check script is present: $vmCheckScript"
}
else {
    Write-Status "WARN" "Ubuntu/VMware environment check script is missing: $vmCheckScript"
}

$python = Get-Command "python" -ErrorAction SilentlyContinue
if ($null -eq $python) {
    Write-Status "WARN" "python was not found on PATH; project verification cannot run from Windows."
}
else {
    $pythonVersion = Invoke-CheckedCommand -FilePath $python.Source -Arguments @("--version") -FailureMessage "python --version failed."
    Write-Status "OK" ($pythonVersion -join " ")

    try {
        Invoke-CheckedCommand -FilePath $python.Source -Arguments @("-c", "import yaml") -FailureMessage "Python yaml import failed." | Out-Null
        Write-Status "OK" "Python PyYAML is available."
    }
    catch {
        Write-Status "WARN" "Python PyYAML is missing; install pyyaml or python3-yaml before running full verification."
    }

    if (!$SkipProjectVerify) {
        $verifyScript = Join-Path $workspace "scripts\verify_project.py"
        Write-Status "INFO" "Running offline project verification."
        Invoke-CheckedCommand -FilePath $python.Source -Arguments @($verifyScript) -FailureMessage "Offline project verification failed." | ForEach-Object {
            Write-Host $_
        }
    }
    else {
        Write-Status "WARN" "Skipped offline project verification."
    }
}

if ($SkipWslCheck) {
    $RosCheckMode = "None"
}
if ($RosCheckMode -eq "Auto") {
    if (Test-CommandAvailable -Name "wsl.exe") {
        $RosCheckMode = "WSL"
    }
    else {
        $RosCheckMode = "VMware"
    }
}

if ($RosCheckMode -eq "None") {
    Write-Status "WARN" "Skipped ROS environment check."
}
elseif ($RosCheckMode -eq "VMware") {
    Write-Status "INFO" "VMware mode selected; guest ROS checks must be run inside the Ubuntu VM."
    Write-Status "INFO" "Copy or share this workspace into the VM, then run:"
    Write-Host "  cd $VmWorkspacePath"
    Write-Host "  bash scripts/bootstrap_vmware_ubuntu.sh"
}
elseif (!(Test-CommandAvailable -Name "wsl.exe")) {
    Write-Status "WARN" "wsl.exe was not found. Use -RosCheckMode VMware for a VMware Ubuntu guest."
}
else {
    Write-Status "INFO" "Checking WSL/ROS command availability."
    $wslScript = @'
set -u
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
  echo ROS_HUMBLE=ok
else
  echo ROS_HUMBLE=missing
fi
command -v ros2 >/dev/null 2>&1 && echo ROS2_CLI=ok || echo ROS2_CLI=missing
command -v colcon >/dev/null 2>&1 && echo COLCON=ok || echo COLCON=missing
command -v python3 >/dev/null 2>&1 && echo PYTHON3=ok || echo PYTHON3=missing
command -v gz >/dev/null 2>&1 && echo GAZEBO_GZ=ok || echo GAZEBO_GZ=missing
command -v ffmpeg >/dev/null 2>&1 && echo FFMPEG=ok || echo FFMPEG=missing
ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1 && echo MOVEIT_MOVE_GROUP=ok || echo MOVEIT_MOVE_GROUP=missing
ros2 pkg prefix moveit_configs_utils >/dev/null 2>&1 && echo MOVEIT_CONFIGS_UTILS=ok || echo MOVEIT_CONFIGS_UTILS=missing
'@
    $wslArgs = @()
    if (![string]::IsNullOrWhiteSpace($WslDistro)) {
        $wslArgs += @("-d", $WslDistro)
    }
    $wslArgs += @("bash", "-lc", $wslScript)

    try {
        $wslOutput = & wsl.exe @wslArgs 2>&1
        foreach ($line in $wslOutput) {
            if ($line -match "=ok$") {
                Write-Status "OK" $line
            }
            elseif ($line -match "=missing$") {
                Write-Status "WARN" $line
            }
            else {
                Write-Status "INFO" $line
            }
        }
    }
    catch {
        Write-Status "WARN" "WSL check failed: $($_.Exception.Message)"
    }
}

Write-Status "OK" "Bootstrap complete."
Write-Status "INFO" "Next Ubuntu build command: cd $VmWorkspacePath && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
