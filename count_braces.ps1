$path = "D:\Proiecte-C++\High-precision-AOCS\src\cpp\aocs_realtime_simulator_integrated.cpp"
$text = Get-Content -Raw $path
$chars = $text.ToCharArray()
$open = ($chars | Where-Object { $_ -eq '{' }).Count
$close = ($chars | Where-Object { $_ -eq '}' }).Count
Write-Output "open={$open} close={$close}"
