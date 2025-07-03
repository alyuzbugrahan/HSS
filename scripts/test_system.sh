#!/bin/bash
# 🧪 Hava Savunma Sistemi Test Scripti
# Jetson Nano'da tüm sistemleri test eder

set -e

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_test() {
    echo -e "${BLUE}🧪 TEST: $1${NC}"
}

print_pass() {
    echo -e "${GREEN}✅ PASS: $1${NC}"
}

print_fail() {
    echo -e "${RED}❌ FAIL: $1${NC}"
}

print_warn() {
    echo -e "${YELLOW}⚠️  WARN: $1${NC}"
}

# Test counters
TESTS_TOTAL=0
TESTS_PASSED=0

run_test() {
    local test_name="$1"
    local test_command="$2"
    
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    print_test "$test_name"
    
    if eval "$test_command" > /dev/null 2>&1; then
        print_pass "$test_name"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        print_fail "$test_name"
        return 1
    fi
}

echo "🎯 Hava Savunma Sistemi - Kapsamlı Test"
echo "========================================"

# ROS2 Environment Test
print_test "ROS2 Environment"
if source /opt/ros/humble/setup.bash 2>/dev/null; then
    print_pass "ROS2 environment loaded"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_fail "ROS2 environment not found"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# Workspace Test
print_test "ROS2 Workspace"
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_pass "Workspace found and sourced"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_fail "Workspace not built - run 'colcon build' first"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# Python Dependencies Test
echo ""
echo "🐍 Python Dependencies Test"
echo "----------------------------"

python_deps=(
    "ultralytics"
    "cv2"
    "numpy"
    "rclpy"
    "Jetson.GPIO"
)

for dep in "${python_deps[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        print_pass "Python module: $dep"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        print_fail "Python module: $dep"
    fi
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
done

# Hardware Tests
echo ""
echo "🔧 Hardware Tests"
echo "----------------"

# GPIO Test
print_test "GPIO Access"
if [ -c /dev/gpiomem ] && groups | grep -q gpio; then
    print_pass "GPIO access available"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_fail "GPIO access not available"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# Camera Test
print_test "Camera Devices"
if ls /dev/video* > /dev/null 2>&1; then
    cameras=$(ls /dev/video* | wc -l)
    print_pass "Found $cameras camera device(s)"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_fail "No camera devices found"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# CUDA Test (if available)
print_test "CUDA Support"
if python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null | grep -q "True"; then
    print_pass "CUDA available"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_warn "CUDA not available (CPU mode only)"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# YOLO Models Test
echo ""
echo "🤖 YOLO Models Test"
echo "------------------"

yolo_models=(
    "Computer_Vision/Mission1/Mission_1.pt"
    "Computer_Vision/Mission2/v2_last.pt"
    "Computer_Vision/Mission3/görev3_demo.pt"
)

for model in "${yolo_models[@]}"; do
    if [ -f "$model" ]; then
        print_pass "Model found: $model"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        print_fail "Model missing: $model"
    fi
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
done

# Configuration Files Test
echo ""
echo "⚙️ Configuration Files Test"
echo "---------------------------"

config_files=(
    "config/camera_config.json"
    "config/motor_config.json"
    "config/firing_config.json"
    "ros2_air_defense/config/air_defense_config.yaml"
)

for config in "${config_files[@]}"; do
    if [ -f "$config" ]; then
        print_pass "Config found: $config"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        print_fail "Config missing: $config"
    fi
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
done

# ROS2 Nodes Test
echo ""
echo "🤖 ROS2 Nodes Test"
echo "------------------"

# Check if nodes can be imported
nodes=(
    "vision_bridge_node.py"
    "motor_controller_node.py"
    "firing_controller_node.py"
    "coordinate_transformer_node.py"
    "air_defense_orchestrator_node.py"
)

for node in "${nodes[@]}"; do
    if [ -f "ros2_air_defense/scripts/$node" ]; then
        if python3 -m py_compile "ros2_air_defense/scripts/$node" 2>/dev/null; then
            print_pass "Node compiles: $node"
            TESTS_PASSED=$((TESTS_PASSED + 1))
        else
            print_fail "Node syntax error: $node"
        fi
    else
        print_fail "Node missing: $node"
    fi
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
done

# Performance Test
echo ""
echo "⚡ Performance Test"
echo "------------------"

# RAM Test
total_ram=$(free -m | awk 'NR==2{printf "%.0f", $2}')
if [ "$total_ram" -gt 3000 ]; then
    print_pass "RAM: ${total_ram}MB (Good)"
    TESTS_PASSED=$((TESTS_PASSED + 1))
elif [ "$total_ram" -gt 2000 ]; then
    print_warn "RAM: ${total_ram}MB (Acceptable)"
else
    print_fail "RAM: ${total_ram}MB (Insufficient)"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# Storage Test
available_space=$(df -h . | awk 'NR==2{print $4}' | sed 's/G//')
if (( $(echo "$available_space > 5" | bc -l) )); then
    print_pass "Available space: ${available_space}G"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    print_fail "Available space: ${available_space}G (Low)"
fi
TESTS_TOTAL=$((TESTS_TOTAL + 1))

# Final Results
echo ""
echo "📊 Test Results"
echo "==============="
echo "Tests Passed: $TESTS_PASSED/$TESTS_TOTAL"

if [ "$TESTS_PASSED" -eq "$TESTS_TOTAL" ]; then
    echo -e "${GREEN}🎉 ALL TESTS PASSED! System ready for deployment.${NC}"
    exit 0
elif [ "$TESTS_PASSED" -gt $((TESTS_TOTAL * 3 / 4)) ]; then
    echo -e "${YELLOW}⚠️  Most tests passed. Check failed tests before proceeding.${NC}"
    exit 1
else
    echo -e "${RED}❌ Multiple test failures. System needs attention.${NC}"
    exit 2
fi 