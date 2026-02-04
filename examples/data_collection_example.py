"""
数据采集示例

这个脚本展示了如何使用DataCollector采集机器人数据
"""

import time
from typing import Annotated
from pathlib import Path
import typer
import signal
import sys

# 添加当前目录到Python路径（这样可以导入data_collection模块）
sys.path.insert(0, str(Path(__file__).parent))

from data_collection.data_collector import DataCollector
from data_collection.collection_config import CollectionConfig
from x2robot import connect

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n\n收到中断信号，正在停止...")
    # 注意：DataCollector 已经注册了信号处理器来清理临时文件
    # 这里直接退出即可，清理工作由 DataCollector 的信号处理器完成
    sys.exit(0)


def create_collection_config_for_quanta_x1() -> CollectionConfig:
    # Do not set SDK mode
    collection_config = CollectionConfig()

    # 采集从臂关节状态（CX002模型）
    collection_config.slave_joint_names = [
        'left_arm_joint_states',
        'right_arm_joint_states',
        'lift_joint_states',
        'left_gripper_joint_states',
        'right_gripper_joint_states',
        'head_joint_states'
    ]
    # action_names 会自动根据 joint_names 生成，也可以手动指定
    collection_config.enable_head_rgb_stream = True # 采集头部RGB视频流
    collection_config.enable_left_arm_rgb_stream = True # 采集左臂RGB视频流
    collection_config.enable_right_arm_rgb_stream = True # 采集右臂RGB视频流
    collection_config.enable_left_arm_end_pose = True # 采集左臂末端位姿
    collection_config.enable_right_arm_end_pose = True # 采集右臂末端位姿
    collection_config.enable_odometry = True # 采集里程计数据
    collection_config.enable_master_arm_data = True # 采集主臂关节状态和末端位姿
    collection_config.enable_wrench_ext_world = True # 采集手腕外力
    collection_config.enable_wrench_ext_local = True # 采集手腕本地力

    return collection_config

def create_collection_config_for_quanta_x2() -> CollectionConfig:
    collection_config = CollectionConfig()
    # 采集从臂关节状态（EX001模型）
    collection_config.slave_joint_names = [
        'left_arm_joint_states',
        'right_arm_joint_states',
        'waist_joint_states',
        'left_gripper_joint_states',
        'right_gripper_joint_states',
        'head_joint_states'
    ]
    # action_names 会自动根据 joint_names 生成，也可以手动指定
    collection_config.enable_head_rgb_stream = True # 采集头部RGB视频流
    collection_config.enable_left_arm_rgb_stream = True # 采集左臂RGB视频流
    collection_config.enable_right_arm_rgb_stream = True # 采集右臂RGB视频流
    collection_config.enable_left_arm_end_pose = True # 采集左臂末端位姿
    collection_config.enable_right_arm_end_pose = True # 采集右臂末端位姿
    collection_config.enable_odometry = True # 采集里程计数据
    # 没有主臂关节状态和末端位姿
    # 没有手腕外力和手腕本地力
    # 有触觉传感器数据
    collection_config.enable_left_gripper_tactile = True # 采集左手指触觉传感器数据
    collection_config.enable_right_gripper_tactile = True # 采集右手指触觉传感器数据
    return collection_config

def create_collection_config_for_desktop() -> CollectionConfig:
    collection_config = CollectionConfig()
    # 采集从臂关节状态（Desktop模型，只有左右臂）
    collection_config.slave_joint_names = [
        'left_arm_joint_states',
        'right_arm_joint_states',
        'left_gripper_joint_states',
        'right_gripper_joint_states'
    ]
    # action_names 会自动根据 joint_names 生成，也可以手动指定
    collection_config.enable_head_rgb_stream = True # 采集头部RGB视频流
    collection_config.enable_left_arm_rgb_stream = True # 采集左臂RGB视频流
    collection_config.enable_right_arm_rgb_stream = True # 采集右臂RGB视频流
    collection_config.enable_left_arm_end_pose = True # 采集左臂末端位姿
    collection_config.enable_right_arm_end_pose = True # 采集右臂末端位姿
    return collection_config

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 连接机器人
    print(f"正在连接机器人 {server}...")
    robot = connect(f"x2://{server}")
    print("✓ 机器人连接成功")

    if robot.get_robot_model() == "EX001":
        collection_config = create_collection_config_for_quanta_x1()
    elif robot.get_robot_model() == "CX002":
        collection_config = create_collection_config_for_quanta_x2()
    elif robot.get_robot_model() == "Desktop":
        collection_config = create_collection_config_for_desktop()
    else:
        raise ValueError(f"Invalid model: {robot.get_robot_model()}, valid models: EX001, CX002, Desktop")


    # 创建数据采集器 - 优化配置以提高性能
    collector = DataCollector(
        robot=robot,
        output_dir="./collected_data",
        target_hz=30,                      # 目标频率（降采样后）
        collection_config=collection_config,
        image_quality=95,                  # JPEG质量
        downsample_joint_states=True,       # 是否降采样关节状态, 建议开启（eg: 500Hz -> target_hz=60Hz）
        use_video_storage=True
    )

    print("\n" + "="*60)
    print("数据采集器已就绪")
    print("="*60)
    print(f"输出目录: {collector.output_dir}")
    print(f"目标频率: {collector.target_hz} Hz")
    print(f"图像存储: {'MP4视频' if collector.use_video_storage else 'JPG图像'}")
    print("\n提示:")
    print("  - 每次调用 start_recording() 会自动启动所有数据采集线程")
    print("  - 调用 stop_recording() 会自动停止所有线程并保存数据")
    print("  - 按 Ctrl+C 可以随时中断程序")
    print("="*60 + "\n")
    
    try:
        # 录制多个episodes
        episode_index = 0
        while True:
            # 显示当前要录制的episode编号（基于已有episodes数量）
            current_episode_num = collector.episode_count
            print(f"\n{'='*60}")
            print(f"准备录制 Episode {current_episode_num}")
            print(f"{'='*60}")
            
            # 询问任务名称
            task_name = input("请输入任务名称 (例如: pick_trash，直接回车使用默认名称): ").strip()
            if not task_name:
                task_name = "pick_trash"
            
            print(f"\n任务名称: {task_name}")
            print("请准备好机器人，按 Enter 开始录制...")
            input()
            
            # 开始录制（会自动启动所有采集线程）
            print("\n" + "="*60)
            print("开始录制...")
            print("="*60)
            collector.start_recording(task=task_name)
            
            print("\n✓ 所有数据采集线程已启动")
            print("正在录制中...")
            print("提示: 执行你的任务，完成后按 Enter 停止录制")
            print("      (或按 Ctrl+C 中断当前录制)\n")
            
            # 每秒打印一次统计信息
            recording_interrupted = False
            while True:
                try:
                    # 非阻塞输入检测
                    import select
                    if select.select([sys.stdin], [], [], 1)[0]:
                        input()  # 读取输入
                        break
                    
                    # 打印统计
                    collector.print_stats()
                except KeyboardInterrupt:
                    print("\n\n收到中断信号，正在停止录制...")
                    recording_interrupted = True
                    break
            
            # 停止录制（会自动停止所有采集线程）
            episode_info = None
            if collector.is_recording:
                episode_info = collector.stop_recording()
            
            if episode_info:
                episode_id = episode_info['episode_id']
                print(f"\n{'='*60}")
                print(f"✓ Episode {episode_id} 录制完成!")
                print(f"{'='*60}")
                print(f"  - Episode ID: {episode_id}")
                print(f"  - 任务名称: {episode_info.get('task', task_name)}")
                print(f"  - 帧数: {episode_info['num_frames']}")
                print(f"  - 时长: {episode_info['duration']:.2f}s")
                print(f"  - 保存路径: {episode_info['episode_dir']}")
            else:
                current_episode_num = collector.episode_count
                print(f"\n⚠️  Episode {current_episode_num} 录制失败")
            
            if recording_interrupted:
                print("\n录制已中断")
                break
            
            # 询问是否继续
            print("\n" + "-"*60)
            continue_recording = input("是否继续录制下一个episode? (y/n，默认n): ").strip().lower()
            if continue_recording != 'y':
                print("停止录制")
                break
            
            episode_index += 1
        
    except KeyboardInterrupt:
        print("\n\n收到中断信号，正在停止...")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保停止录制（如果还在录制中）
        if collector.is_recording:
            print("\n正在停止录制...")
            collector.stop_recording()
        
        print("\n" + "="*60)
        print("数据采集已完成!")
        print("="*60)
        print(f"总共录制了 {collector.episode_count} 个episodes")
        print(f"数据保存在: {collector.output_dir}")
        print(f"\n转换为LeRobot格式:")
        print(f"python3 tools/convert_to_lerobot.py \\")
        print(f"    --input-dir {collector.output_dir} \\")
        print(f"    --output-dir ./lerobot_data \\")
        print(f"    --repo-id my_robot/dataset \\")
        print(f"    --robot-type {robot.get_robot_model()} \\")
        print(f"    --use-videos")


if __name__ == "__main__":
    typer.run(main)

