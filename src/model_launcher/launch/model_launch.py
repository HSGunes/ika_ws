from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paketinizin 'share' dizininin yolunu alın
    package_share_directory = get_package_share_directory('model_launcher')

    # Model dosyanızın yolunu tanımlayın
    # 'best.pt' dosyasının 'model_launcher/models/' içinde olduğundan emin olun
    model_path_default = os.path.join(package_share_directory, 'models', 'best.pt')

    # Esneklik için launch argümanlarını tanımlayın
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=model_path_default,
        description='YOLOv8 model dosyasının yolu (.pt)'
    )
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/ika/camera_rgb/image_raw',
        description='RGB kamera görüntüsü için topic'
    )
    detection_image_topic_arg = DeclareLaunchArgument(
        'detection_image_topic',
        default_value='/ika/object_detections_image',
        description='Anotasyonlu tespit görüntülerinin yayınlanacağı topic'
    )
    display_image_arg = DeclareLaunchArgument(
        'display_image',
        default_value='true', # Boolean parametreler için 'true' veya 'false' string olarak kullanılmalı
        description='Tespit görüntüsünün OpenCV kullanılarak gösterilip gösterilmeyeceği'
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5', # Float parametreler için string olarak kullanılmalı
        description='Nesne tespiti için güven eşiği'
    )
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.45', # Float parametreler için string olarak kullanılmalı
        description='Non-Maximum Suppression için IOU eşiği'
    )

    # Başlatılacak düğümü tanımlayın
    object_detector_node = Node(
        package='model_launcher', # Kendi paket adınız
        # ÖNEMLİ: Bu, setup.py'daki 'console_scripts' içinde tanımlayacağınız yürütülebilir isimle eşleşmeli
        executable='object_detector_node_exec', # <--- Bunu setup.py'da tanımlayacağız
        name='object_detector_node', # Düğümün ROS grafiklerindeki adı
        output='screen', # Düğümün çıktısını terminalde göster
        parameters=[ # Düğüme gönderilecek parametreler
            {'model_path': LaunchConfiguration('model_path')},
            {'camera_topic': LaunchConfiguration('camera_topic')},
            {'detection_image_topic': LaunchConfiguration('detection_image_topic')},
            {'display_image': LaunchConfiguration('display_image')},
            {'confidence_threshold': LaunchConfiguration('confidence_threshold')},
            {'iou_threshold': LaunchConfiguration('iou_threshold')},
        ]
    )

    # LaunchDescription objesini döndürerek tüm eylemleri tanımlayın
    return LaunchDescription([
        model_path_arg,
        camera_topic_arg,
        detection_image_topic_arg,
        display_image_arg,
        confidence_threshold_arg,
        iou_threshold_arg,
        object_detector_node
    ])