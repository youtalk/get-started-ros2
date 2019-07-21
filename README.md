---
layout: page
title: オンラインリソース
---

## サンプルコードのセットアップ

### 1章 ROS1ツアー

誌面より

> 次項からステップバイステップで実装していくROS1デモパッケージ `hello_world` のソースコードはオンラインリソース
>
> [https://github.com/youtalk/get-started-ros2/tree/release/ros1/hello_world](https://github.com/youtalk/get-started-ros2/tree/release/ros1/hello_world)
>
> にビルド可能な形で全て保存されています。
> 本文では紙面の都合上、ライセンスやインクルード文などを省略し、ソースコードも一部のみを抜粋して記載しています。ソースコード全体をご覧になりたい場合には、こちらをご参照ください。
> ライセンス条項に関しては、まとめて付録に記載しています。
>
> サンプルコードのセットアップ方法は以下の通りです。適宜、本文と照らし合わせながら読み進めていってください。
>
> ```shell
> $ cd ~/ && git clone https://github.com/youtalk/get-started-ros2.git
> $ cd get-started-ros2 && git submodule update --init
> $ mkdir ~/ros1 && cd ~/ros1
> $ ln -s ~/get-started-ros2/ros1 src
> $ rosdep install --from-paths src --ignore-src -r -y
> $ catkin init
> $ catkin build
> $ catkin source
> ```

### 2章 ROS2の開発環境セットアップ

誌面より

> 次章からステップバイステップで実装していくROS2デモパッケージ `hello_world` および、4章、7章で使用するパッケージのソースコードはオンラインリソース
>
> [https://github.com/youtalk/get-started-ros2/tree/release/ros2](https://github.com/youtalk/get-started-ros2/tree/release/ros2)
>
> 以下にビルド可能な形で全て保存されています。
> 本文では紙面の都合上、ライセンスやインクルード文などを省略し、ソースコードも一部のみを抜粋して記載しています。ソースコード全体をご覧になりたい場合には、こちらをご参照ください。
> ライセンス条項に関しては、まとめて付録に記載しています。
>
> サンプルコードのセットアップ方法は以下の通りです。適宜、本文と照らし合わせながら読み進めていってください。
>
> ```shell
> $ cd ~/ && git clone https://github.com/youtalk/get-started-ros2.git
> $ cd get-started-ros2 && git submodule update --init
> $ mkdir ~/ros2 && cd ~/ros2
> $ ln -s ~/get-started-ros2/ros2 src
> $ rosdep install --from-paths src --ignore-src -r -y
> $ colcon build
> $ . ~/ros2/install/setup.bash
> ```
