import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import math
import argparse
import sys

# カスタムサービス (事前にビルドしておく)
from rf_sim_interfaces.srv import GetRxPower

class RFAntennaPublisher(Node):
    def __init__(self, 
                 antenna_type="omnidirectional", 
                 freq=2.4e9, 
                 tx_power_dbm=0.0,
                 yaw_deg=0.0, 
                 pitch_deg=0.0):
        super().__init__('rf_antenna_publisher')

        # ---------------------------
        # (A) パラメータ設定
        # ---------------------------
        self.antenna_type = antenna_type
        self.freq_hz = freq
        self.tx_power_dbm = tx_power_dbm
        self.reflection_coef = 0.7  # 地面反射係数(線形)
        self.num_points = 100000      # ランダム生成する点の数
        self.max_radius = 50.0      # 最大伝搬距離

        # アンテナ位置
        self.real_antenna_pos = np.array([0.0, 0.0, 1.0])
        self.image_antenna_pos = np.array([0.0, 0.0, -1.0])  # イメージアンテナ (反射)

        # アンテナの向き (Yaw/Pitch) → メインローブ軸ベクトル self.antenna_dir
        self.antenna_dir = self.compute_antenna_direction(yaw_deg, pitch_deg)

        self.get_logger().info(f"[RFAntennaPublisher] type={antenna_type}, freq={freq}, tx={tx_power_dbm} dBm")
        self.get_logger().info(f"Yaw={yaw_deg} deg, Pitch={pitch_deg} deg, dir={self.antenna_dir}")

        # ---------------------------
        # (B) ポイントクラウドパブリッシャ
        # ---------------------------
        self.publisher_ = self.create_publisher(PointCloud2, 'rf_field', 10)
        self.timer = self.create_timer(0.5, self.publish_rf_field)

        # ---------------------------
        # (C) サービスサーバ
        # ---------------------------
        self.srv_server = self.create_service(GetRxPower, 'get_rx_power', self.get_rx_power_callback)

    # ---------------------------
    # (A1) アンテナ方向ベクトルを計算
    # ---------------------------
    def compute_antenna_direction(self, yaw_deg, pitch_deg):
        """
        z軸正方向(0,0,1) を初期向きとし、Yaw(Y軸回り), Pitch(Y軸回り)で回転して
        アンテナのメインローブ軸となる単位ベクトルを返す。
         - yaw_deg: Z軸回り (heading)
         - pitch_deg: Y軸回り
        好みで roll_deg(X軸回り) を追加してもよい。
        """
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)

        # 初期ベクトル: z軸
        v = np.array([0.0, 0.0, 1.0], dtype=float)

        # 回転行列 Rz(yaw)
        Rz = np.array([
            [ math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [ math.sin(yaw_rad),  math.cos(yaw_rad), 0],
            [               0,                 0,    1],
        ], dtype=float)

        # 回転行列 Ry(pitch)
        Ry = np.array([
            [ math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [ 0,                   1,                  0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)],
        ], dtype=float)

        # 合成回転 R = Rz * Ry (順序は用途に応じて調整)
        R = Rz @ Ry

        # 回転後ベクトル
        rotated = R @ v
        norm_val = np.linalg.norm(rotated)
        if norm_val < 1e-12:
            return np.array([0,0,1], dtype=float)
        return rotated / norm_val

    # =======================================================
    # (B) ポイントクラウドをパブリッシュ
    # =======================================================
    def publish_rf_field(self):
        # 1) 実アンテナ(直接波)
        x1, y1, z1, p_db1 = self.generate_points(antenna_pos=self.real_antenna_pos, reflection_lin=1.0)

        # 2) イメージアンテナ(反射波)
        x2, y2, z2, p_db2 = self.generate_points(antenna_pos=self.image_antenna_pos, reflection_lin=self.reflection_coef)

        # 合体
        x_all = np.concatenate((x1, x2))
        y_all = np.concatenate((y1, y2))
        z_all = np.concatenate((z1, z2))
        p_db_all = np.concatenate((p_db1, p_db2))

        # z<0 は削除 (地面下なので電波なし)
        mask = (z_all >= 0)
        x_all = x_all[mask]
        y_all = y_all[mask]
        z_all = z_all[mask]
        p_db_all = p_db_all[mask]

        # dB(-100～0) → 0.0～1.0 に正規化
        intensity = (p_db_all + 100.0) / 100.0
        intensity = np.clip(intensity, 0.0, 1.0)

        # PointCloud2 作成
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(x_all)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = 16  # (x, y, z, intensity) 4つのfloat32
        msg.row_step = msg.point_step * len(x_all)

        msg.fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data_list = []
        for i in range(len(x_all)):
            data_list.append(struct.pack('ffff', x_all[i], y_all[i], z_all[i], intensity[i]))
        msg.data = b"".join(data_list)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {len(x_all)} points")

    def generate_points(self, antenna_pos, reflection_lin):
        """
        ランダムに点 (r,theta,phi) を生成し、その点での受信電力[dB]を計算
        (アンテナの向き self.antenna_dir や指向性を考慮)
        """
        # (1) ランダムに (r, theta, phi) 生成
        r     = np.random.uniform(0.5, self.max_radius, self.num_points)
        theta = np.random.uniform(0.0, np.pi, self.num_points)
        phi   = np.random.uniform(0.0, 2.0*np.pi, self.num_points)

        # (2) 直交座標に変換 (アンテナ位置をオフセット)
        x = r*np.sin(theta)*np.cos(phi) + antenna_pos[0]
        y = r*np.sin(theta)*np.sin(phi) + antenna_pos[1]
        z = r*np.cos(theta)             + antenna_pos[2]

        # (3) 相対ベクトル rel
        rel_x = x - antenna_pos[0]
        rel_y = y - antenna_pos[1]
        rel_z = z - antenna_pos[2]
        rr = np.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        rr = np.clip(rr, 1e-12, None)

        # (4) アンテナ向き self.antenna_dir との角度を計算
        #     dot = rel・dir
        dot_vals = rel_x*self.antenna_dir[0] + rel_y*self.antenna_dir[1] + rel_z*self.antenna_dir[2]
        cos_vals = dot_vals / (rr * np.linalg.norm(self.antenna_dir))  # dirは単位ベクトルなのでnorm=1だが、一応…
        cos_vals = np.clip(cos_vals, -1.0, 1.0)
        theta_ant = np.arccos(cos_vals)  # 0 <= theta_ant <= π

        # (5) 指向性ゲイン [dBi]
        gain_db = self.compute_gain_db(theta_ant)

        # (6) 反射係数 (線形→dB)
        reflection_db = 10.0 * math.log10(reflection_lin)

        # (7) フリースペース伝搬損失 FSPL(dB)
        fspl_db = 20.0*np.log10(rr) + 20.0*np.log10(self.freq_hz) - 147.55

        # (8) 受信電力 p_rx_db
        p_rx_db = self.tx_power_dbm + gain_db + reflection_db - fspl_db
        return x, y, z, p_rx_db

    def compute_gain_db(self, theta):
        """
        アンテナタイプによる指向性ゲイン [dBi] を返す
         - omnidirectional: 0 dBi
         - directional: cos²(θ) (線形 → dB)
         - yagi: ガウシアンビーム
        """
        if self.antenna_type == "omnidirectional":
            return np.zeros_like(theta)

        elif self.antenna_type == "directional":
            g_lin = np.cos(theta)**2
            g_db = 10.0*np.log10(np.clip(g_lin, 1e-12, None))
            return g_db

        elif self.antenna_type == "yagi":
            theta_0 = np.pi/6.0  # 30度
            g_lin = np.exp(- (theta/theta_0)**2)
            g_db = 10.0*np.log10(np.clip(g_lin, 1e-12, None))
            return g_db

        else:
            self.get_logger().warn("Unknown antenna type -> use omnidirectional(0dBi)")
            return np.zeros_like(theta)

    # =======================================================
    # (C) サービス (x,y,z) -> 受信電力[dB]
    # =======================================================
    def get_rx_power_callback(self, request, response):
        """
        任意の座標 (x,y,z) での「直接波 + 反射波」を合成した受信電力 [dB] を計算して返す。
        z<0 なら -999dB とする。
        """
        xq, yq, zq = request.x, request.y, request.z

        # 地面下の場合は電波なし
        if zq < 0:
            response.rx_db = -999.0
            return response

        # (1) 直接波(実アンテナ)
        direct_db = self.compute_rx_power_db_for_point(xq, yq, zq, self.real_antenna_pos, 1.0)
        # (2) 反射波(イメージアンテナ)
        refl_db   = self.compute_rx_power_db_for_point(xq, yq, zq, self.image_antenna_pos, self.reflection_coef)

        # (3) パワー合成 (線形加算 -> dB)
        p_direct_lin = 10.0**(direct_db/10.0)
        p_refl_lin   = 10.0**(refl_db/10.0)
        p_total_lin  = p_direct_lin + p_refl_lin
        p_total_db   = 10.0 * math.log10(p_total_lin)

        response.rx_db = p_total_db
        return response

    def compute_rx_power_db_for_point(self, x, y, z, antenna_pos, reflection_lin):
        """
        1点 (x,y,z) での受信電力 [dB] を計算 (アンテナ指向性 + FSPL + reflection)
        """
        rel_x = x - antenna_pos[0]
        rel_y = y - antenna_pos[1]
        rel_z = z - antenna_pos[2]
        rr = math.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        if rr < 1e-12:
            rr = 1e-12

        # アンテナ向き self.antenna_dir との角度
        dot_val = rel_x*self.antenna_dir[0] + rel_y*self.antenna_dir[1] + rel_z*self.antenna_dir[2]
        cos_val = dot_val / (rr * np.linalg.norm(self.antenna_dir))
        cos_val = max(-1.0, min(1.0, cos_val))
        theta = math.acos(cos_val)

        # 指向性ゲイン [dBi]
        gain_db = 0.0
        if self.antenna_type == "omnidirectional":
            gain_db = 0.0
        elif self.antenna_type == "directional":
            g_lin = (math.cos(theta))**2
            gain_db = 10.0*math.log10(max(g_lin, 1e-12))
        elif self.antenna_type == "yagi":
            theta_0 = math.pi/6.0
            g_lin = math.exp(- (theta/theta_0)**2)
            gain_db = 10.0*math.log10(max(g_lin, 1e-12))

        # 反射係数 (線形→dB)
        reflection_db = 10.0 * math.log10(reflection_lin)

        # FSPL(dB)
        fspl_db = 20.0*math.log10(rr) + 20.0*math.log10(self.freq_hz) - 147.55

        # 受信電力 [dB]
        p_rx_db = self.tx_power_dbm + gain_db + reflection_db - fspl_db
        return p_rx_db


def main():
    parser = argparse.ArgumentParser(description="RF Sim with orientation + reflection + service")
    parser.add_argument('--antenna_type', type=str, default="omnidirectional",
                        choices=["omnidirectional", "directional", "yagi"],
                        help="Antenna model type")
    parser.add_argument('--freq', type=float, default=2.4e9, help="Frequency [Hz]")
    parser.add_argument('--tx_power_dbm', type=float, default=0.0, help="Tx power [dBm]")
    parser.add_argument('--yaw_deg', type=float, default=0.0, help="Yaw around Z-axis [deg]")
    parser.add_argument('--pitch_deg', type=float, default=0.0, help="Pitch around Y-axis [deg]")
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = RFAntennaPublisher(
        antenna_type=args.antenna_type,
        freq=args.freq,
        tx_power_dbm=args.tx_power_dbm,
        yaw_deg=args.yaw_deg,
        pitch_deg=args.pitch_deg
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
