@echo off
setlocal
cd /d "%~dp0"
powershell -NoProfile -Command "if (-not (Get-NetTCPConnection -LocalPort 8788 -State Listen -ErrorAction SilentlyContinue)) { Start-Process -FilePath 'D:\codex\.cad_env\Scripts\python.exe' -ArgumentList 'D:\codex\Cable_driven_massage_github_20260812\tools\y42_twin_server.py' -WorkingDirectory 'D:\codex\Cable_driven_massage_github_20260812\tools' -WindowStyle Hidden }; Start-Process 'http://127.0.0.1:8788'"
endlocal
