from setuptools import find_packages, setup

package_name = 'llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vision_interface'],
    zip_safe=True,
    maintainer='mozihe',
    maintainer_email='zhujunheng2005@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'match_info_subscriber = llm.match_info_subscriber:main',  # 注册可执行文件
        ],
    },
)
