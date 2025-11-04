#!/bin/bash
# 多机器人中转系统监控脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认机器人名称
ROBOT_NAME=${1:-robot1}

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  多机器人中转系统监控${NC}"
echo -e "${BLUE}  机器人: ${ROBOT_NAME}${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# 检查中转节点是否运行
echo -e "${YELLOW}📡 检查中转节点...${NC}"
if ros2 node list | grep -q "topic_relay_node"; then
    echo -e "${GREEN}✅ 话题中转节点运行中${NC}"
else
    echo -e "${RED}❌ 话题中转节点未运行${NC}"
    echo -e "${YELLOW}   启动命令: ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=${ROBOT_NAME}${NC}"
fi

if ros2 node list | grep -q "tf_relay_node"; then
    echo -e "${GREEN}✅ TF中转节点运行中${NC}"
else
    echo -e "${YELLOW}⚠️  TF中转节点未运行${NC}"
fi

echo ""

# 检查本地话题
echo -e "${YELLOW}📋 检查本地话题...${NC}"
LOCAL_TOPICS=("/odom" "/scan" "/amcl_pose" "/cmd_vel")
for topic in "${LOCAL_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✅ ${topic}${NC}"
    else
        echo -e "${RED}❌ ${topic} (未找到)${NC}"
    fi
done

echo ""

# 检查全局话题
echo -e "${YELLOW}🌐 检查全局话题...${NC}"
GLOBAL_TOPICS=(
    "/${ROBOT_NAME}/odom"
    "/${ROBOT_NAME}/scan"
    "/${ROBOT_NAME}/amcl_pose"
    "/${ROBOT_NAME}/cmd_vel_status"
)
for topic in "${GLOBAL_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✅ ${topic}${NC}"
    else
        echo -e "${RED}❌ ${topic} (未找到)${NC}"
    fi
done

echo ""

# 检查话题频率
echo -e "${YELLOW}📊 检查话题频率 (5秒采样)...${NC}"
if ros2 topic list | grep -q "^/odom$"; then
    HZ=$(timeout 5 ros2 topic hz /odom 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$HZ" ]; then
        echo -e "${GREEN}  /odom: ${HZ} Hz${NC}"
    else
        echo -e "${YELLOW}  /odom: 无数据${NC}"
    fi
fi

if ros2 topic list | grep -q "^/${ROBOT_NAME}/odom$"; then
    HZ=$(timeout 5 ros2 topic hz /${ROBOT_NAME}/odom 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$HZ" ]; then
        echo -e "${GREEN}  /${ROBOT_NAME}/odom: ${HZ} Hz${NC}"
    else
        echo -e "${YELLOW}  /${ROBOT_NAME}/odom: 无数据${NC}"
    fi
fi

echo ""

# 检查进程资源使用
echo -e "${YELLOW}💻 检查资源使用...${NC}"
if pgrep -f "topic_relay" > /dev/null; then
    CPU=$(ps aux | grep topic_relay | grep -v grep | awk '{print $3}')
    MEM=$(ps aux | grep topic_relay | grep -v grep | awk '{print $4}')
    echo -e "${GREEN}  CPU: ${CPU}%${NC}"
    echo -e "${GREEN}  内存: ${MEM}%${NC}"
fi

echo ""

# 检查网络状态（多机器人）
echo -e "${YELLOW}🌐 检查网络状态...${NC}"
echo -e "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未设置}"
echo -e "  主机名: $(hostname)"
echo -e "  IP地址: $(hostname -I | awk '{print $1}')"

echo ""

# 快速数据测试
echo -e "${YELLOW}🧪 快速数据测试...${NC}"
if ros2 topic list | grep -q "^/odom$"; then
    echo -e "  测试 /odom 数据..."
    if timeout 2 ros2 topic echo /odom --once > /dev/null 2>&1; then
        echo -e "${GREEN}  ✅ /odom 数据正常${NC}"
    else
        echo -e "${RED}  ❌ /odom 无数据${NC}"
    fi
fi

if ros2 topic list | grep -q "^/${ROBOT_NAME}/odom$"; then
    echo -e "  测试 /${ROBOT_NAME}/odom 数据..."
    if timeout 2 ros2 topic echo /${ROBOT_NAME}/odom --once > /dev/null 2>&1; then
        echo -e "${GREEN}  ✅ /${ROBOT_NAME}/odom 数据正常 (中转成功)${NC}"
    else
        echo -e "${RED}  ❌ /${ROBOT_NAME}/odom 无数据 (中转失败)${NC}"
    fi
fi

echo ""
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  监控完成${NC}"
echo -e "${BLUE}================================${NC}"
echo ""
echo -e "${YELLOW}💡 提示:${NC}"
echo -e "   - 运行完整测试: python3 test/quick_test.py --robot-name ${ROBOT_NAME}"
echo -e "   - 查看节点信息: ros2 node info /topic_relay_node"
echo -e "   - 查看话题列表: ros2 topic list | grep robot"
echo ""
