# Build/run standalone EXP5 and proposed-rerun synthetic tests in a fresh isolated directory.
# Uses the existing Release/IPOPT debug-CRT ABI; main.cpp helpers compile with formal main() excluded.
[CmdletBinding()]
param([switch]$BuildOnly)

$ErrorActionPreference = 'Stop'
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$validationRoot = Join-Path $projectDirectory 'x64\RerunValidation'
$validationDirectory = Join-Path $validationRoot ((Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0, 8))
$resolvedValidation = [IO.Path]::GetFullPath($validationDirectory)
if (-not $resolvedValidation.StartsWith($validationRoot + '\', [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Validation directory escaped its dedicated build root.'
}
$vcvars = 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
$ipoptDirectory = 'C:\CodeEnv\Ipopt-3.14.19-win64-msvs2022-mdd'
if (-not (Test-Path -LiteralPath $vcvars)) { throw "Missing compiler setup: $vcvars" }
New-Item -ItemType Directory -Path $resolvedValidation | Out-Null
$executable = Join-Path $resolvedValidation 'localization_tests.exe'
$compilerArguments = @('/nologo', '/std:c++17', '/permissive-', '/EHsc', '/utf-8', '/O2', '/Gy', '/MDd',
    '/DNDEBUG', '/D_DEBUG', '/D_ITERATOR_DEBUG_LEVEL=2', '/D_CONSOLE', '/DIL_STD',
    '/I"E:\Mydvtools\boost_1_89_0"',
    ('/I"' + (Join-Path $ipoptDirectory 'include\coin-or') + '"'),
    ('/I"' + (Join-Path $projectDirectory 'packages\nlohmann.json.3.12.0\build\native\include') + '"'))
$sources = @((Join-Path $PSScriptRoot 'localization_tests.cpp'))
foreach ($sourceName in @('EntityDefinition.cpp', 'Convexrelaxationandrounding.cpp',
    'MatchingSQP.cpp', 'HungarianMatching.cpp', 'SADA_algorithm.cpp')) {
    $sources += Join-Path $projectDirectory $sourceName
}
foreach ($source in $sources) { $compilerArguments += '"' + $source + '"' }
$compilerArguments += @(('/Fe"' + $executable + '"'), ('/Fo"' + $resolvedValidation + '\\"'),
    '/link', '/OPT:REF', ('/LIBPATH:"' + (Join-Path $ipoptDirectory 'lib') + '"'),
    '/LIBPATH:"E:\Mydvtools\boost_1_89_0\stage\lib"', 'ipopt.dll.lib')
$compileCommand = 'call "' + $vcvars + '" >nul && cl.exe ' + ($compilerArguments -join ' ')
Write-Output "Isolated test build: $resolvedValidation"
Push-Location -LiteralPath $resolvedValidation
try {
    & cmd.exe /d /c $compileCommand
    if ($LASTEXITCODE -ne 0) { throw "Standalone build failed with exit code $LASTEXITCODE" }
    if (-not $BuildOnly) {
        $originalSearchPath = $env:PATH
        try {
            $env:PATH = (Join-Path $ipoptDirectory 'bin') + ';' + $originalSearchPath
            # Keep the nested backup/candidate test paths below legacy Windows MAX_PATH.
            & $executable (Join-Path $resolvedValidation 's')
            if ($LASTEXITCODE -ne 0) { throw "Synthetic validation failed with exit code $LASTEXITCODE" }
        } finally { $env:PATH = $originalSearchPath }
    }
} finally { Pop-Location }
Write-Output "Validation artifacts retained: $resolvedValidation"
