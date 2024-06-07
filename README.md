# KXR_HAC_Software

![jpg](https://github.com/rt-net/KXR_HAC_Software/assets/103564180/5ae96d94-1070-4794-992b-afbceb43f62b)

近藤科学社製KXR-L2を自律化し，株式会社アールティが主催する大会 Humanoid Autonomous Challenge (HAC)に参加するためのソフトウェアです．<br>

Humanoid Autonomous Challenge<br>
(https://www.rt-shop.jp/blog/archives/10714)<br>

KXR-L2にRaspberryPi Zero 2W, WEBカメラ，IMUなどを増設した以下のハードウェア上で使用することを想定しています．他のロボットハードウェアで使用する際は，動作中に呼び出すモーションやパラメータを適宜編集してください．<br>
(https://github.com/rt-net/KXR_HAC_Software.git)

## ディレクトリ構成
```vision```
```motion_control```
```motion_planning```
```tmp```
```HTN_sample```
```sample```

<pre>
├── README.md
├── Rcb4BaseLib.py
├── parameterfile.py
├── HTN_planner.py
├── run_HTN_planner.py
├── vision
│   └── vision_library.py
├── motion_control
│   └── motion_control_library.py
├── motion_planning
│   └── motion_planning_library.py
├── tmp
│   ├── dist.csv
│   ├── left_corner_template.jpg
│   ├── left_corner_template_gray.jpg
│   ├── left_corner_template_wide_gray.jpg
│   ├── mtx.csv
│   ├── right_corner_template.jpg
│   ├── right_corner_template_gray.jpg
│   └── right_corner_template_wide_gray.jpg
├── HTN_sample
│   ├── planning_library_sample.py
│   └── run_HTN_planner_sample.py
└── sample
    ├── detect_goal_sample.py
    ├── htn_test.py
    ├── motion_control_library_sample.py
    ├── motion_planning_test.py
    └── vision_library_test.py
 </pre>

## ライセンス
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="クリエイティブ・コモンズ・ライセンス" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />この 作品 は <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">クリエイティブ・コモンズ 表示 4.0 国際 ライセンス</a>の下に提供されています。

## 作者
TaikiTsuno

## 参考
近藤科学　KXR アドバンスセットA  商品ページ<br>
(https://kondo-robot.com/product/03158)<br>
アールティ　ヒューマノイドロボットブログ<br>
(https://rt-net.jp/humanoid/)

