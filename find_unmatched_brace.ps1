$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$lines = Get-Content $path
$stack = New-Object System.Collections.Generic.List[object]
$lineNo = 0
foreach ($line in $lines) {
  $lineNo++
  for ($i = 0; $i -lt $line.Length; $i++) {
    $ch = $line[$i]
    if ($ch -eq '{') {
      $stack.Add([pscustomobject]@{ line=$lineNo; col=$i+1 })
    } elseif ($ch -eq '}') {
      if ($stack.Count -gt 0) {
        $stack.RemoveAt($stack.Count-1)
      } else {
        Write-Output "unmatched_close at line=$lineNo col=$($i+1)"
      }
    }
  }
}
if ($stack.Count -gt 0) {
  Write-Output "unmatched_open_count=$($stack.Count)"
  $stack | Select-Object -Last 5 | ForEach-Object { Write-Output "unmatched_open at line=$($_.line) col=$($_.col)" }
} else {
  Write-Output "all_matched"
}
