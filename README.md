# BeanAccReader
LightBlue Beanの加速度センサデータをローパスフィルタ付きで読み出し

無次元カットオフ0.2（サンプリング10Hzならカットオフ1Hz）の２次バタワースローパスフィルタを直接II形転置形で実装
