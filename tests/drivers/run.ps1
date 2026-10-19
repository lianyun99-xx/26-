param([string]$Compiler = 'C:/Program Files/MATLAB/R2024b/sys/lcc64/lcc64/bin/lcc64.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path "$PSScriptRoot/../..").Path
$buildDir = Join-Path $projectRoot '.build/driver-regression'
New-Item -ItemType Directory -Force $buildDir | Out-Null
$compilerRoot = Split-Path (Split-Path $Compiler)
if (!(Test-Path $Compiler)) { throw 'Native LCC compiler missing; pass -Compiler pointing to lcc64.exe.' }
$sources = @('tests/drivers/regression.c','Device/Motors/3508/3508_driver.c',
  'Device/BMI088/BMI088Middleware.c','Device/BMI088/BMI088driver.c','Device/BMI088/Attitude_solution.c')
$objects = @()
Push-Location $projectRoot
try {
  foreach ($source in $sources) {
    $object = Join-Path $buildDir ([IO.Path]::GetFileNameWithoutExtension($source) + '.obj')
    & $Compiler -noregistrylookup -c "-I$compilerRoot/include64" '-Itests/drivers/stubs' `
      '-IDevice/Motors/3508' '-IDevice/BMI088' '-ICore/Inc' "-Fo$object" $source
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
    $objects += $object
  }
  $executable = Join-Path $buildDir 'regression.exe'
  & "$compilerRoot/bin/lcclnk64.exe" "-L$compilerRoot/lib64" -subsystem console -o $executable @objects
  if ($LASTEXITCODE -ne 0) { throw 'Native test link failed' }
  & $executable
  if ($LASTEXITCODE -ne 0) { throw 'Driver regression failed' }
  python tests/drivers/check_config.py
  if ($LASTEXITCODE -ne 0) { throw 'Configuration regression failed' }
} finally { Pop-Location }
