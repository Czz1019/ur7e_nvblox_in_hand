"""D435 specialization: ROS robot/sensor orchestration, wheel-only mapping backend."""
from pathlib import Path
import shlex,math,yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def resolve_input_topics(base, supplied, use_sim_time):
    """Keep simulation defaults; match the real driver's camera/camera namespace.

    Explicit topic arguments always win. CameraInfo remains paired with its own
    optical image; this does not align depth or change any calibration.
    """
    real = dict(depth_topic='/camera/camera/depth/image_rect_raw',
                depth_info_topic='/camera/camera/depth/camera_info',
                color_topic='/camera/camera/color/image_raw',
                color_info_topic='/camera/camera/color/camera_info')
    return {key: supplied.get(key) or (base[key] if use_sim_time else topic)
            for key, topic in real.items()}


def setup(context):
    def text(k):return context.launch_configurations[k]
    def flag(k):
        v=text(k).lower()
        if v not in ('true','false'):raise ValueError(k+' must be true or false')
        return v=='true'
    share=Path(get_package_share_directory('ur7e_nvblox_in_hand'))
    base=yaml.safe_load((share/'config/base.yaml').read_text())['/**']['ros__parameters']
    if text('profile') not in ('realtime','reference'):raise ValueError('Invalid profile')
    if flag('start_scene') and (not flag('use_sim_time') or flag('launch_camera') or text('rosbag')):raise ValueError('Gazebo requires simulation time and exclusive input source')
    if flag('launch_camera') and (flag('use_sim_time') or text('rosbag')):raise ValueError('Physical D435 requires use_sim_time=false and no bag')
    actions=[]
    if flag('start_scene'):
        scene=Path(get_package_share_directory('ur7e_d435_gazebo'))/'launch/scene.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene)),launch_arguments={k:text(k) for k in ['gazebo_gui','camera_width','camera_height','scene_variant','sim_camera_reliability']}.items()))
    if flag('launch_camera'):
        size=text('camera_width')+'x'+text('camera_height')+'x30'
        actions.append(Node(package='realsense2_camera',executable='realsense2_camera_node',name='camera',namespace='camera',parameters=[str(share/'config/d435.yaml'),{'depth_module.depth_profile':size,'rgb_camera.color_profile':size,'publish_tf':True}],output='screen'))
    if text('rosbag'):
        bag=Path(text('rosbag')).expanduser().resolve()
        if not bag.exists() or not flag('use_sim_time'):raise ValueError('Bag missing or use_sim_time=false')
        actions.append(ExecuteProcess(cmd=['ros2','bag','play',str(bag)]+shlex.split(text('rosbag_args')),output='screen'))
    config=[str(share/'config/base.yaml'),str(share/'config'/(text('profile')+'.yaml'))]
    if text('mapper_config'):
        file=Path(text('mapper_config')).expanduser().resolve()
        if not file.is_file():raise ValueError('mapper_config missing')
        config.append(str(file))
    overrides={'use_sim_time':flag('use_sim_time')}
    # The bundled simulator offers reliable images by default. Preserve the
    # sensor-data default for real cameras and external/older bag publishers.
    if flag('start_scene') and not text('input_qos'):
        overrides['input_qos']='reliable' if text('sim_camera_reliability')=='reliable' else 'sensor_data'
    for key,default in base.items():
        value=text(key)
        if value=='':continue
        if isinstance(default,bool):overrides[key]=flag(key)
        elif isinstance(default,int):overrides[key]=int(value)
        elif isinstance(default,float):
            n=float(value)
            if not math.isfinite(n):raise ValueError(key+' must be finite')
            overrides[key]=n
        else:overrides[key]=value
    topics=resolve_input_topics(base,{key:text(key) for key in
        ('depth_topic','depth_info_topic','color_topic','color_info_topic')},flag('use_sim_time'))
    overrides.update(topics)
    mode=text('rviz_mode')
    esdf_mode=overrides.get('esdf_mode',base['esdf_mode'])
    if mode=='auto':mode='2d' if esdf_mode=='2d' else 'diagnostic'
    if mode not in ('mesh','diagnostic','2d'):raise ValueError('Invalid rviz_mode')
    mapper=Node(package='ur7e_nvblox_in_hand',executable='mapper_node',name='ur7e_nvblox',parameters=config+[overrides],output='screen')
    actions+=[mapper,RegisterEventHandler(OnProcessExit(target_action=mapper,on_exit=[EmitEvent(event=Shutdown(reason='Required Python Mapper exited'))]))]
    if flag('run_rviz'):actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz'/(mode+'.rviz'))],parameters=[{'use_sim_time':flag('use_sim_time')}],remappings=[(base[key],topic) for key,topic in topics.items()],output='screen'))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('ur7e_nvblox_in_hand'))
    base=yaml.safe_load((share/'config/base.yaml').read_text())['/**']['ros__parameters']
    orchestration=dict(sim_camera_reliability='reliable',scene_variant='lab',start_scene='true',launch_camera='false',use_sim_time='true',gazebo_gui='true',run_rviz='true',camera_width='640',camera_height='480',profile='realtime',mapper_config='',rviz_mode='auto',rosbag='',rosbag_args='')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in orchestration.items()]+[DeclareLaunchArgument(k,default_value='') for k in base]+[OpaqueFunction(function=setup)])
