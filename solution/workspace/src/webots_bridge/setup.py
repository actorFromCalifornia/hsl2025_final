from setuptools import setup

package_name = 'webots_bridge'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/cobra_flex_demo.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/webots_bridge.urdf']))
data_files.append(('share/' + package_name + '/objects/backgrounds/protos', [
    'objects/backgrounds/protos/TexturedBackground.proto',
    'objects/backgrounds/protos/TexturedBackgroundLight.proto'
    ]))
data_files.append(('share/' + package_name + '/objects/floors/protos', [
    'objects/floors/protos/Floor.proto',
    'objects/floors/protos/RectangleArena.proto',
    ]))
data_files.append(('share/' + package_name + '/objects/solids/protos', [
    'objects/solids/protos/SolidBox.proto',
    'objects/solids/protos/SolidPipe.proto',
    'objects/solids/protos/SolidRoundedBox.proto',
    'objects/solids/protos/SolidTorus.proto',
    ]))
data_files.append(('share/' + package_name + '/bounding_objects/protos', [
    'bounding_objects/protos/PipeBoundingObject.proto',
    'bounding_objects/protos/TorusBoundingObject.proto',
    ]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/TrafficCone.proto',
    'protos/CobraFlex.proto',
    'protos/GeneratedMaze.proto',
    'protos/GeneratedCones.proto',
    ]))
data_files.append(('share/' + package_name + '/protos/textures', [
    'protos/textures/cone.jpg',
    ]))
data_files.append(('share/' + package_name + '/protos/meshes', [
    'protos/meshes/robot_body.stl',
    'protos/meshes/wheel_cobra.stl',
    ]))
data_files.append(('share/' + package_name + '/appearances/protos', [
    'appearances/protos/BrushedAluminium.proto',
    'appearances/protos/Parquetry.proto',
    ]))
data_files.append(('share/' + package_name + '/appearances/protos/textures/brushed_aluminium', [
    'appearances/protos/textures/brushed_aluminium/brushed_aluminium_base_color.jpg',
    'appearances/protos/textures/brushed_aluminium/brushed_aluminium_normal.jpg',
    'appearances/protos/textures/brushed_aluminium/brushed_aluminium_roughness.jpg',
    ]))
data_files.append(('share/' + package_name + '/appearances/protos/textures/parquetry', [
    'appearances/protos/textures/parquetry/chequered_parquetry_base_color.jpg',
    'appearances/protos/textures/parquetry/chequered_parquetry_normal.jpg',
    'appearances/protos/textures/parquetry/chequered_parquetry_occlusion.jpg',
    'appearances/protos/textures/parquetry/chequered_parquetry_roughness.jpg',
    'appearances/protos/textures/parquetry/dark_strip_parquetry_base_color.jpg',
    'appearances/protos/textures/parquetry/dark_strip_parquetry_normal.jpg',
    'appearances/protos/textures/parquetry/dark_strip_parquetry_occlusion.jpg',
    'appearances/protos/textures/parquetry/light_strip_parquetry_base_color.jpg',
    'appearances/protos/textures/parquetry/light_strip_parquetry_normal.jpg',
    'appearances/protos/textures/parquetry/light_strip_parquetry_roughness.jpg',
    'appearances/protos/textures/parquetry/mosaic_parquetry_base_color.jpg',
    'appearances/protos/textures/parquetry/mosaic_parquetry_normal.jpg',
    'appearances/protos/textures/parquetry/mosaic_parquetry_occlusion.jpg',
    'appearances/protos/textures/parquetry/mosaic_parquetry_roughness.jpg',
    ]))
data_files.append(('share/' + package_name + '/default/worlds/textures', [
    'default/worlds/textures/tagged_wall.jpg',
    ]))
data_files.append(('share/' + package_name + '/default/worlds/textures/cubic', [
    'default/worlds/textures/cubic/mountains_back.hdr',
    'default/worlds/textures/cubic/mountains_back.jpg',
    'default/worlds/textures/cubic/mountains_bottom.hdr',
    'default/worlds/textures/cubic/mountains_bottom.jpg',
    'default/worlds/textures/cubic/mountains_front.hdr',
    'default/worlds/textures/cubic/mountains_front.jpg',
    'default/worlds/textures/cubic/mountains_left.hdr',
    'default/worlds/textures/cubic/mountains_left.jpg',
    'default/worlds/textures/cubic/mountains_right.hdr',
    'default/worlds/textures/cubic/mountains_right.jpg',
    'default/worlds/textures/cubic/mountains_top.hdr',
    'default/worlds/textures/cubic/mountains_top.jpg',
    ]))
data_files.append(('share/' + package_name + '/devices/slamtec/protos', [
    'devices/slamtec/protos/RpLidarA2.proto',
    ]))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavel Skorynin',
    maintainer_email='pavel.skorynin@yandex.ru',
    description='',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_driver = webots_bridge.webots_driver:main',
        ],
    },
)