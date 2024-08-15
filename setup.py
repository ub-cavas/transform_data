from setuptools import find_packages, setup

package_name = 'transform_data'

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
    maintainer='beast',
    maintainer_email='beast@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_forward = transform_data.data_conversion:main',
            'enu = transform_data.ENU_Converter:main',
            "fast_lio = transform_data.fast_lio_mapping_tree:main",
            "save_pcd = transform_data.save_pcd:main",
        ],
    },
)


# import os
# from glob import glob
# from setuptools import setup

# package_name = 'transform_data'

# setup(data_files=[(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'remap_data.launch.py')))])