param([int]$stopLine)
$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$lines = Get-Content $path
$open=0;$close=0;$ln=0
foreach($line in $lines){$ln++;$chars=$line.ToCharArray();foreach($c in $chars){if($c -eq '{'){$open++}elseif($c -eq '}'){$close++}}if($ln -eq $stopLine){break}}
Write-Output "lines=$stopLine open=$open close=$close bal=$($open-$close)"