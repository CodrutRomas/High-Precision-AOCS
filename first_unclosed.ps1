$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$lines = Get-Content $path
$bal=0;$ln=0;$firstUnclosed=-1
foreach($line in $lines){$ln++;foreach($c in $line.ToCharArray()){if($c -eq '{'){$bal++;if($bal -eq 1 -and $firstUnclosed -lt 0){$firstUnclosed=$ln}}elseif($c -eq '}'){$bal--}}}
Write-Output "final_balance=$bal first_unclosed_line=$firstUnclosed"