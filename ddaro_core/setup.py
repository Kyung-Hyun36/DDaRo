from setuptools import find_packages, setup

package_name = 'ddaro_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치파일 추가 
        ('share/' + package_name + '/launch', ['launch/ddaro_core.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyun',
    maintainer_email='hyun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = ddaro_core.navigator_node:main',
        ],
    },
)
