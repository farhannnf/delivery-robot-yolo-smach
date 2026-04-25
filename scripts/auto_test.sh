#!/bin/bash

echo "=== TESTING 30X WITH AUTO ERROR LOGGING ==="
echo "Run,Target,Timestamp" > ~/test_log.csv

# Test Pak Yani (10x)
for i in {1..10}; do
    echo "=== Run $i: Pak Yani ==="
    START=$(date +%s)
    rosparam set /delivery/recipient_name "Pak Yani"
    rosparam set /delivery/return_to_home true
    
    rosrun ta2_farhan simple_state_machine.py > ~/log_run_${i}_yani.txt 2>&1
    
    END=$(date +%s)
    DURATION=$((END - START))
    
    echo "$i,Pak Yani,$START" >> ~/test_log.csv
    
    sleep 5
done

# Test Bu Eka (10x)
for i in {11..20}; do
    echo "=== Run $i: Bu Eka ==="
    START=$(date +%s)
    rosparam set /delivery/recipient_name "Bu Eka"
    rosparam set /delivery/return_to_home true
    
    rosrun ta2_farhan simple_state_machine.py > ~/log_run_${i}_eka.txt 2>&1
    
    END=$(date +%s)
    DURATION=$((END - START))
    
    echo "$i,Bu Eka,$START" >> ~/test_log.csv
    
    sleep 5
done

# Test Pak Dosen A (10x)
for i in {21..30}; do
    echo "=== Run $i: Pak Dosen A ==="
    START=$(date +%s)
    rosparam set /delivery/recipient_name "Pak Dosen A"
    rosparam set /delivery/return_to_home true
    
    rosrun ta2_farhan simple_state_machine.py > ~/log_run_${i}_dosen.txt 2>&1
    
    END=$(date +%s)
    DURATION=$((END - START))
    
    echo "$i,Pak Dosen A,$START" >> ~/test_log.csv
    
    sleep 5
done

echo "=== TESTING SELESAI ==="
echo "Log files di ~/log_run_*.txt"
echo "Summary di ~/test_log.csv"
