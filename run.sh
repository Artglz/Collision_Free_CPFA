#!/bin/bash

# ARGoS Multiple Run Script
# This script runs the simulation 10 times with different random seeds
# and extracts the "Resource Collected:" lines from the output

CONFIG_FILE="experiments/Clustered_CPFA_r40_tag256_16by16_TEST.xml" # go to this file change the robot numbers: if 48 robots, then change name below accordingly.
OUTPUT_FILE="results_18_mins_64_robots.txt" # change just the number of robots, ex: results_18_mins_48_robots.txt
TEMP_CONFIG="temp_64.xml" # here too, ex: temp_48.xml

# Array of random seeds to use
SEEDS=(123456 789012 345678 901234 567890 135791 246802 864209 751936 428573 
       613579 982034 704321 319864 580246 162738 947120 830591 204869 678345  
       793624 115320 926481 307159 481206 693018 859473 374829 142607 620591)

# Clear previous results
> "$OUTPUT_FILE"

echo "Starting 20 simulation runs with different random seeds..."
echo "Results will be saved to: $OUTPUT_FILE"
echo ""

for i in {0..19}; do # change 19 to 29 if you want 30 simulations
    SEED=${SEEDS[$i]}
    RUN_NUMBER=$((i + 1))
    
    echo "Run $RUN_NUMBER: Using random seed $SEED"
    
    # Create temporary config file with new random seed
    sed "s/random_seed=\"[0-9]*\"/random_seed=\"$SEED\"/" "$CONFIG_FILE" > "$TEMP_CONFIG"
    
    # Run ARGoS simulation and capture output
    echo "=== Run $RUN_NUMBER (Seed: $SEED) ===" >> "$OUTPUT_FILE"
    
    # Run the simulation and filter for "Resource Collected:" lines
    argos3 -c "$TEMP_CONFIG" 2>&1 | grep "Resource Collected:" | tee -a "$OUTPUT_FILE"
    
    echo "" >> "$OUTPUT_FILE"
    
    echo "Run $RUN_NUMBER completed"
    echo ""
done

# Clean up temporary file
rm -f "$TEMP_CONFIG"

echo ""
echo "Detailed results saved in: $OUTPUT_FILE"