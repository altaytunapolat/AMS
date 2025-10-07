python .\double_ams_plot_V2.py `
>>   --sensor-port COM4 --sensor-baud 115200 `
>>    --psu1 COM20 --psu2 COM21 --addr 6 --pv 12 `
>>    --k1 4.0 --k2 4.0 --sign1 -1 --sign2 -1 `
>>   --soll1 0 --soll2 0 `
>>   --sk1 0.2 --sk2 0.2 `
>>    --dirate 8 --lpf-alpha 0.2 `
>>   --write-sleep-ms 2 `
>>   --tel-period 0.3 `
>>    --plot --plot-history 60 --plot-fps 25 `
>>    --csv COMPENDATA.txt --hotkeys