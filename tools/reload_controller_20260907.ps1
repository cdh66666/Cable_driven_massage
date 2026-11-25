$ErrorActionPreference = 'Stop'
$project = 'D:\codex\Cable_driven_massage_github_20260812'
$expectedControllerId = 20704
$expectedCommands = @('"C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\y42_pc_direct_web.py', '"D:\codex\Cable_driven_massage_github_20260812\.venv-hardware\Scripts\python.exe" tools\y42_pc_direct_web.py')
$recordStamp = Get-Date -Format 'yyyyMMdd_HHmmss'
function Assert-OldController {
    $listeners = @(Get-NetTCPConnection -LocalPort 8787 -State Listen)
    if ($listeners.Count -ne 1 -or $listeners[0].OwningProcess -ne $expectedControllerId) { throw 'Controller listener changed; no process stopped.' }
    $process = Get-CimInstance Win32_Process -Filter "ProcessId=$expectedControllerId"
    if ($process.CommandLine -notin $expectedCommands) { throw ('Controller command mismatch; no process stopped: ' + $process.CommandLine) }
}
Assert-OldController
& "$project\.venv-hardware\Scripts\python.exe" -m py_compile "$project\tools\y42_pc_direct_web.py"
if ($LASTEXITCODE -ne 0) { throw 'New controller compilation failed' }
$null = Invoke-RestMethod 'http://127.0.0.1:8787/api/command' -Method Post -ContentType 'application/json' -Body '{"action":"stop","confirmed":true}' -TimeoutSec 5
$confirmed = $false
for ($attempt=0; $attempt -lt 20; $attempt++) {
    Start-Sleep -Milliseconds 500
    $state = Invoke-RestMethod 'http://127.0.0.1:8787/api/auto' -TimeoutSec 5
    if ($state.emergency_stop_pending -eq $false -and $state.emergency_stop_acknowledged -eq $true -and $state.auto -eq 'idle' -and $state.manual_test_recording.active -eq $false) {
        $confirmed = $true
        break
    }
}
if (-not $confirmed) { throw 'Four-axis STOP/DISABLE was not confirmed; controller left running.' }
$status = Invoke-RestMethod 'http://127.0.0.1:8787/api/status' -TimeoutSec 8
if ($status.status_deferred -eq $true -or @($status.motors).Count -ne 4) { throw 'Fresh four-axis status unavailable; controller left running.' }
foreach ($motor in $status.motors) {
    if ($motor.online -ne $true -or $null -eq $motor.flags -or ($motor.flags -band 1) -ne 0 -or $null -eq $motor.rpm -or [Math]::Abs($motor.rpm) -gt 0.1) { throw 'An axis is not confirmed disabled and stopped; controller left running.' }
}
Assert-OldController
$proof = @{ prior_process=$expectedControllerId; stop_confirmed=$true; motors=$status.motors; source_sha256=(Get-FileHash "$project\tools\y42_pc_direct_web.py" -Algorithm SHA256).Hash; time_utc=[DateTime]::UtcNow.ToString('o') }
$proof | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$project\qa\twin\controller-reload-$recordStamp.json" -Encoding utf8
Stop-Process -Id $expectedControllerId -ErrorAction Stop
$newProcess = Start-Process -FilePath "$project\.venv-hardware\Scripts\python.exe" -ArgumentList 'tools\y42_pc_direct_web.py' -WorkingDirectory $project -WindowStyle Hidden -RedirectStandardOutput "$project\qa\twin\controller-reload-$recordStamp.stdout.log" -RedirectStandardError "$project\qa\twin\controller-reload-$recordStamp.stderr.log" -PassThru
Write-Output "Started controller launcher PID $($newProcess.Id); four motors were confirmed disabled before restart."
