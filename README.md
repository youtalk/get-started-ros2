---
layout: page
title: オンラインリソース
---

## サンプルコード

本書のサンプルコードは以下のGitHubレポジトリで管理されています。Apache License 2.0の下、ご自由にお使いください。

[https://github.com/youtalk/get-started-ros2](https://github.com/youtalk/get-started-ros2)

サンプルコードの動作確認は、以下のROSディストリビューションで行なっております。

- [ROS 2 Humble Hawksbill](https://github.com/youtalk/get-started-ros2/tree/humble)
- [ROS 2 Jazzy Jalisco](https://github.com/youtalk/get-started-ros2/tree/jazzy)

サンプルコードは紙面の文字数制限の都合上、1行あたり72文字以内で記述しています。ROS2公式の `ament_uncrustify` の整形ルールとは合致しない点、ご留意ください。

## セットアップ

### 3章 開発環境セットアップ

誌面より

> 次章からステップバイステップで実装していくROS 2デモパッケージ `hello_world` および5章、8章で使用するパッケージのソースコードは、次のオンラインリソースにビルド可能な形ですべて保存されています。

>
> [https://github.com/youtalk/get-started-ros2/tree/main/src](https://github.com/youtalk/get-started-ros2/tree/main/src)
>
> 本書では紙面の都合上、ライセンスやインクルード文などを省略し、ソースコードも一部のみを抜粋して記載しています。ソースコード全体をご覧になりたい場合には、こちらを参照してください。
> ライセンス条項に関しては、まとめて付録に記載しています。
>
> サンプルコードのセットアップ方法は以下の通りです。適宜、本文と照らし合わせながら読み進めていってください。
>
> ```shell
> $ cd ~/ && git clone https://github.com/youtalk/get-started-ros2.git
> $ cd get-started-ros2
> $ rosdep install --from-paths src --ignore-src -r -y
> $ colcon build
> $ . ~/get-started-ros2/install/setup.bash
> ```
