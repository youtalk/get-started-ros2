from setuptools import setup

package_name = 'examples_rclpy_topics'

setup(
    name=package_name,  # パッケージ名
    version='0.6.3',  # バージョン番号
    packages=[package_name],  # ソースコードのディレクトリ
    data_files=[  # ソースコード以外のファイル
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # 依存Python3モジュール
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Yutaka Kondo',
    maintainer_email='yutaka.kondo@youtalk.jp',
    keywords=['ROS'],
    classifiers=[  # PyPIの分類情報
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of publishers/subscribers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],  # テストフレームワーク名
    entry_points={  # 実行コマンド名とその呼び出し先
        'console_scripts': [
            'publisher = ' + package_name + '.publisher:main',
            'subscriber = ' + package_name + '.subscriber:main',
            'composed = ' + package_name + '.composed:main',
        ],
    },
)
