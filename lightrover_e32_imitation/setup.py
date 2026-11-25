from glob import glob

from setuptools import setup

package_name = 'lightrover_e32_imitation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vstone',
    maintainer_email='infodesk@vstone.co.jp',
    description='ライトローバーE32 模倣学習サンプル',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'il_recorder = lightrover_e32_imitation.recorder:main',
            'il_cmd_mux = lightrover_e32_imitation.cmd_mux:main',
            'il_policy = lightrover_e32_imitation.policy_node:main',
            'il_train = lightrover_e32_imitation.train:main',
        ],
    },
)
