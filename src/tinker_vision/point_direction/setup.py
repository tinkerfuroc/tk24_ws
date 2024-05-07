from setuptools import find_packages, setup

package_name = 'point_direction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youwang',
    maintainer_email='787488028@qq.com',
    description='Tinker vision package for point direction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_direction = point_direction.point_direction:main'
        ],
    },
)
