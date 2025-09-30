$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$start=129; $end=228
$lines = Get-Content $path
$bal=0
for($i=$start;$i -le $end;$i++){
 $line=$lines[$i-1]
 $opens=($line.ToCharArray() | Where-Object { $_ -eq '{' }).Count
 $closes=($line.ToCharArray() | Where-Object { $_ -eq '}' }).Count
 $bal+=$opens-$closes
 if($opens -ne 0 -or $closes -ne 0){ Write-Output ("{0,4}: +{1} -{2} bal={3} :: {4}" -f $i,$opens,$closes,$bal,$line.Trim()) }
}
Write-Output "segment_balance=$bal"