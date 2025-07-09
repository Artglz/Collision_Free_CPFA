#!/bin/bash

# === Configurable Parameters ===
CONFIG_FILE="experiments/Clustered_CPFA_r40_tag256_16by16_TEST.xml"
NUM_ROBOTS=48
NUM_RUNS=50  # <-- Change this value to control how many simulations to run

OUTPUT_FILE="results_18_mins_${NUM_ROBOTS}_robots.txt"
TEMP_CONFIG="temp_${NUM_ROBOTS}.xml"

# === Clear previous results ===
> "$OUTPUT_FILE"

echo "Starting $NUM_RUNS simulation runs with different random seeds..."
echo "Results will be saved to: $OUTPUT_FILE"
echo ""

# === Main Simulation Loop ===
for ((i = 1; i <= NUM_RUNS; i++)); do
    SEED=$((RANDOM % 1000001))  # Range: 0 to 100000
    echo "Run $i: Using random seed $SEED"
    
    # Create temporary config file with the new random seed
    sed "s/random_seed=\"[0-9]*\"/random_seed=\"$SEED\"/" "$CONFIG_FILE" > "$TEMP_CONFIG"
    
    # Run ARGoS simulation and filter for "Resource Collected:" lines
    echo "=== Run $i (Seed: $SEED) ===" >> "$OUTPUT_FILE"
    argos3 -c "$TEMP_CONFIG" 2>&1 | grep "Resource Collected:" | tee -a "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
    
    echo "Run $i completed"
    echo ""
done

# === Clean Up ===
rm -f "$TEMP_CONFIG"

echo ""
echo "Detailed results saved in: $OUTPUT_FILE"
