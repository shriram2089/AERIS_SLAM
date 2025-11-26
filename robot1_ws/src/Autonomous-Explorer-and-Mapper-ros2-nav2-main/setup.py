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
            'explorer = custom_explorer.explorer:main',
             'adv_explorer = custom_explorer.advanced_explorer:main',
             'basic_explorer = custom_explorer.basic_explorer:main',
             
             'dumb_explorer = custom_explorer.dumb_explorer:main',
             'dumb_explorer_adv = custom_explorer.dumb_explorer_adv:main',
             'dumb_explorer2 = custom_explorer.dumb_explorer2:main',
             
             
             'scan_tester = custom_explorer.scan_tester:main',
             
        ],
    },
)
