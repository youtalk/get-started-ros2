---
layout: page
title: FAQ
---

## macOS Mojaveでの開発環境セットアップに失敗する

macOS Mojaveでの開発環境セットアップ後に、ROS2関連コマンドを実行すると以下のようなエラーが表示される可能性があります。

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
