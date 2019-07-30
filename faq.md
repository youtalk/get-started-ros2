---
layout: page
title: FAQ
---

よくある質問とその回答（FAQ）を記載します。もし、その他に本書の内容に関する質問がある場合には、

[https://github.com/youtalk/get-started-ros2/issues](https://github.com/youtalk/get-started-ros2/issues)

にご投稿ください。ROS2一般に関する質問などは上記には投稿せず、ROS2公式の質問サイトをご利用ください。

- 英語 [https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:ros2/](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:ros2/)
- 日本語 [https://discourse.ros.org/c/local/japan](https://discourse.ros.org/c/local/japan)

## macOS Mojaveでの開発環境セットアップに失敗する

macOS Mojaveでの開発環境セットアップ後に、ROS2関連コマンドを実行すると以下のようなエラーが表示される可能性があります。

### libtinyxml2のバージョン問題

```shell
$ ros2 --help
Failed to load entry point 'test': dlopen(/Users/youtalk/ros/dashing/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: /usr/local/opt/tinyxml2@6.2.0/lib/libtinyxml2.6.dylib
  Referenced from: /Users/youtalk/ros/dashing/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.dylib
  Reason: image not found
```

以上のような `libtinyxml2` のインストールバージョンの違いは、 `Homebew` のGitレポジトリを操作して古いバージョンをインストールする必要があります。
まず、インストール済みの `tinyxml2` のバージョンを確認します。

```shell
$ brew info tinyxml2
tinyxml2: stable 7.0.1 (bottled), HEAD
Improved tinyxml (in memory efficiency and size)
http://grinninglizard.com/tinyxml2
/usr/local/Cellar/tinyxml2/7.0.1 (11 files, 155.2KB) *
  Poured from bottle on 2018-12-06 at 10:42:45
From: https://github.com/Homebrew/homebrew-core/blob/master/Formula/tinyxml2.rb
...
```

ご覧のように `7.X.X` がインストールされている場合、現在のmacOS用のROS2バイナリでは動きません。 `Formula` を修正して、 `6.2.0` をインストールするように変更しましょう。

```shell
$ cd /usr/local/Homebrew/Library/Taps/homebrew/homebrew-core/Formula
$ git checkout da9a96f093a57eb8ac7c9470c1ec04fd1823a37a tinyxml2.rb
$ brew unlink tinyxml2
$ brew install tinyxml2
$ brew switch tinyxml2 6.2.0
```

### libPocoFoundationのバージョン問題

```shell
$ ros2 --help
Failed to load entry point 'test': dlopen(/Users/youtalk/ros/dashing/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: /usr/local/opt/poco/lib/libPocoFoundation.60.dylib
  Referenced from: /Users/youtalk/ros/dashing/lib/librosidl_typesupport_c.dylib
  Reason: image not found
```

以上のような `libPocoFoundation` のインストールバージョンの違いはシンボリックリンクで対処できます。
まず、インストール済みの `libPocoFoundation` のバージョンを確認します。

```shell
$ ls -l /usr/local/opt/poco/lib/libPocoFoundation.*
-rw-r--r--  1 youtalk  staff  1704520  7 30 13:16 /usr/local/opt/poco/lib/libPocoFoundation.62.dylib
lrwxr-xr-x  1 youtalk  staff       26  7  2 13:23 /usr/local/opt/poco/lib/libPocoFoundation.dylib -> libPocoFoundation.62.dylib
```

この例では `62` でしたが、要求されるバージョンは `60` です。`62` から `60` へのシンボリックリンクを作成することでエラーが回避できます。

```shell
$ ln -s /usr/local/opt/poco/lib/libPocoFoundation.62.dylib /usr/local/opt/poco/lib/libPocoFoundation.60.dylib
```
