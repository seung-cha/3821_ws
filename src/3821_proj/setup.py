from setuptools import setup, find_packages
package_name = '3821_proj'

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
    maintainer='seung',
    maintainer_email='seunghwancha5857@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        [
            '3821_planner = 3821_proj.path_planner:main',
            '3821_follower = 3821_proj.path_follower:main',
        ],
    },
)
