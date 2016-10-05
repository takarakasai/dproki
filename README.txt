
# このリポジトリに関して

このリポジトリではP4アーム向けの身体モデル編集ツール（およびライブラリlibp4model）を提供します.
シェルからツールを実行するか、ソフトウェアにライブラリを組み込むことでP4モデルの変態をサポートすることができます.

## モデル編集ライブラリ

-- ヘッダファイル : p4model.h
-- ライブラリファイル : libp4model.so

### API一覧

p4model_open の第1引数にて取得したポインタを使いまわします.
resdir, filepathなどの仕様はobjファイル形式に準拠しています（はずです）。

errno_t p4model_open (void** inst, const char* resdir, const char* filepath);

errno_t p4model_close (void* inst);

errno_t p4model_change_model(void* inst, P4ARM_LENGTH length, double wrist_pitch_value/*[deg]*/);

errno_t p4model_saveas (void* inst, const char* filepath);

errno_t p4model_add_camera(void* inst, const char *lnk_name, camera_sensor *param);

### 仕様例

```
void* inst;
p4model_open(&inst, "/home/kasai/smrpkg/resource", "/obj/p4arm/p4arm.obj");

p4model_change_model(inst, P4ARM_SHORT /* or P4ARM_MIDDLE or P4ARM_LONG*/, 90.0 /*[deg]*/);

                      /* name    width height vang hang  near  far    x   y    z      r   p   y */
camera_sensor param = {"CameraR", 1200, 800, 40.0, 40.0, 0.05, 10.0, {0.0,0.0,0.0}, {30.0,0.0,0.0}};
p4model_add_camera(inst, "CAMERA_YAW", &param);

p4model_saveas(inst, "/home/kasai/smrpkg/resource_modified");

p4model_close(inst);
```

## モデル編集ツール

### コマンド一覧

p4model_chg        : 既存のP4アームモデルを変態(スライド軸、受動回転軸を変化)させたモデルを生成します.

p4model_add_sensor : 既存のP4アームモデルにセンサを取り付けたモデルを生成します.
                     現状ではカメラセンサのみに対応しています.

### 引数

p4model_chg <res_dirpath> <filepath> <save_dirpath> <S|M|L> <rot_value[deg]>
  res_dirpath : リソースディレクトリパス
  filepath    : objファイルへのリソースディレクトリからの相対パス
  save_dirpath: 保存先のリソースディレクトリパス
  S|M|L       : スライド軸の状態
                   S : Short, M: Middle, L: Long
  rot_value   : 回転軸の角度

p4model_add_sensor <res_dirpath> <filepath> <save_dirpath> linkname Camera name width height vang hang near far x y z r p y
  res_dirpath : リソースディレクトリパス
  filepath    : objファイルへのリソースディレクトリからの相対パス
  save_dirpath: 保存先のリソースディレクトリパス
  linkname    : センサを取り付けるリンクの名前
  Camera      : センサの種別（現状ではCameraセンサのみ)
  name        : センサの名前
  width height vang hang near far x y z r p y  : センサのパラメータ(objファイル形式に準拠)

### Reasenote

--- 2016/09/30 
branch: develop
commit: 1d5aeba63d95fb0642ba9c79ce8d43924ea066f4
comment:

以下コミットの直視鏡モデルに対応

commit 5c6b6fcce75fcd7584c8156b64d5822c69e2bd80
Author: Yasuhiro Matsuda <Yasuhiro.Matsuda@jp.sony.com>
Date:   Thu Sep 29 18:13:26 2016 +0900

    update P4ARM_ENDO_STRAIGHT (2016.09 masspro)

