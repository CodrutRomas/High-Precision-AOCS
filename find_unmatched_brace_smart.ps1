$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$lines = Get-Content $path
$stack = New-Object System.Collections.Generic.List[object]
$lineNo = 0
$inString = $false
$escape = $false
foreach ($line in $lines) {
  $lineNo++
  # strip // comments (not perfect; ignores // inside strings, but we handle strings below)
  $processed = ""
  $i = 0
  while ($i -lt $line.Length) {
    $ch = $line[$i]
    if (-not $inString -and $i -lt ($line.Length-1) -and $line[$i] -eq '/' -and $line[$i+1] -eq '/') {
      break
    }
    if ($ch -eq '"' -and -not $escape) {
      $inString = -not $inString
    }
    $escape = ($ch -eq '\\' -and -not $escape)
    $processed += $ch
    $i++
  }
  for ($j = 0; $j -lt $processed.Length; $j++) {
    $c = $processed[$j]
    if (-not $inString) {
      if ($c -eq '{') { $stack.Add([pscustomobject]@{ line=$lineNo; col=$j+1 }) }
      elseif ($c -eq '}') {
        if ($stack.Count -gt 0) { $stack.RemoveAt($stack.Count-1) } else { Write-Output "unmatched_close at line=$lineNo col=$($j+1)" }
      }
    }
  }
}
if ($stack.Count -gt 0) {
  Write-Output "unmatched_open_count=$($stack.Count)"
  $stack | Select-Object -Last 10 | ForEach-Object { Write-Output "unmatched_open at line=$($_.line) col=$($_.col)" }
} else {
  Write-Output "all_matched"
}
