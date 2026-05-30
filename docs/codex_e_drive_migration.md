# Codex E Drive Migration Note

Target root: `E:\codex`

Already staged on E drive:

- `E:\codex\home\.codex`
- `E:\codex\home\.agents`
- `E:\codex\home\.cache\codex-runtimes`
- `E:\codex\AppData\Local\OpenAI`
- `E:\codex\AppData\Local\Packages\OpenAI.Codex_2p2nqsd0c76g0`
- `E:\codex\AppData\Local\Packages\OpenAI.ChatGPT-Desktop_2p2nqsd0c76g0`
- `E:\codex\vscode\extensions\openai.chatgpt-*`
- `E:\codex\vscode\extensions\hiztam.codex-history-viewer-2.4.1`
- `E:\codex\AppData\Roaming\Code\User\globalStorage\hiztam.codex-history-viewer`

The user-level environment variable has been set:

```powershell
CODEX_HOME=E:\codex\home\.codex
```

Current Codex, VS Code, MCP, and OpenAI helper processes are still using some C-drive paths, so the final path switch must be done after closing Codex and VS Code.

Run this from a normal PowerShell window after closing Codex and VS Code:

```powershell
powershell -ExecutionPolicy Bypass -File E:\codex\migrate_codex_to_e.ps1
```

If the script reports remaining Codex/OpenAI processes and you accept stopping them automatically:

```powershell
powershell -ExecutionPolicy Bypass -File E:\codex\migrate_codex_to_e.ps1 -ForceStop
```

After the script finishes, the original C-drive paths are replaced by directory junctions pointing to `E:\codex`, and the original C-drive folders are moved into an E-drive backup folder named `E:\codex\original_location_backups_<timestamp>`.

The Windows Store application binary under `C:\Program Files\WindowsApps\OpenAI.Codex_*` is system-managed and is not moved by this script. The script migrates Codex user data, runtime cache, VS Code extension files, and MCP tool folders.
