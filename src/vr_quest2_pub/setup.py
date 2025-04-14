from setuptools import find_packages, setup

package_name = 'vr_quest2_pub'  # 包名

setup(
    name=package_name,
    version='0.0.1',  # 更新版本号
    packages=find_packages(exclude=['test']),
    # package_data={"encapsulated_oculusReader":["encapsulated_oculusReader/*"]},
    # include_package_data=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    # install_requires=['setuptools','ppadb'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h666',
    maintainer_email='liuhuxian@gmail.com',
    description='A ROS2 publisher for VR_quest2, publishing to pos_cmd topic.',  # 更新描述
    license='MIT',  # 更新许可证
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vr_pub = vr_quest2_pub.publisher_node:main',  # 确保指向正确的模块和函数
        ],
    },
)