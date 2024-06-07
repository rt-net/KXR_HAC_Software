# KXR_HAC_Software
近藤科学社製KXR-L2を自律化し，株式会社アールティが主催する大会 Humanoid Autonomous Challenge (HAC)に参加するためのソフトウェアです．

Humanoid Autonomous Challenge
(https://www.rt-shop.jp/blog/archives/10714)

KXR-L2にRaspberryPi Zero 2W, WEBカメラ，IMUなどを増設した以下のハードウェア上で使用することを想定しています．他のロボットハードウェアで使用する際は，動作中に呼び出すモーションやパラメータを適宜編集してください．
(https://github.com/rt-net/KXR_HAC_Software.git)

## フォルダ構成
```vision```
```motion_control```
```motion_planning```
```tmp```
```HTN_sample```
```sample```

❯ tree -a -I "node_modules|.next|.git|.pytest_cache|static" -L 2