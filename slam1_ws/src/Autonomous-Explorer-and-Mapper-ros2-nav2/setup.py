from setuptools import find_packages, setup

package_name = 'custom_explorer'

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
    maintainer='unknown',
    maintainer_email='unknown@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer2 = custom_explorer.explorer:main',
            'explorer = custom_explorer.explorer_ns:main',
            'explorer_lite = custom_explorer.explorer_lite:main', 
            'swarm_explorer = custom_explorer.swarm_explorer_ns:main',
            'global_swarm_explorer = custom_explorer.global_swarm_explorer:main',
            
            'adv_explorer = custom_explorer.adv_explorer_ns:main',
            'global_adv_explorer = custom_explorer.global_adv_explorer:main',
            
            'adv_explorer2 = custom_explorer.adv_explorer_2:main',
            
            
            
        ],
    },
)
