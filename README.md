# gnss_preprocessing

![image](https://github.com/Arcanain/gnss_preprocessing/assets/52307432/4c48a288-4dc6-44d8-b03e-6482577bbfa8)

![image](https://github.com/Arcanain/gnss_preprocessing/assets/52307432/33e85a3d-7d6e-4d6e-bf28-0ee3e07f4d26)

# Algorithm

## ECEF Coordinates

The Cartesian coordinates with the Earth's center as the origin are called ECEF (Earth-Centered, Earth-Fixed) coordinates. The conversion formula can be written in a programming language style as follows:

- `Lat`: Latitude
- `Lon`: Longitude
- `High`: Altitude (elevation)
- `N`: Geoid height
- `PI`: Pi (π)
- `Xe`, `Ye`, `Ze`: ECEF coordinates

```plaintext
f = 1 / 298.257223563;
a = 6378137;
e = sqrt(f * (2 - f));
R = a / sqrt(1 - e^2 * sin(Lat * PI / 180)^2);
Xe = (R + High + N) * cos(Lat * PI / 180) * cos(Lon * PI / 180);
Ye = (R + High + N) * cos(Lat * PI / 180) * sin(Lon * PI / 180);
Ze = (R * (1 - e^2) + High + N) * sin(Lat * PI / 180);
```

## ENU Coordinates (Local Tangent Plane Coordinates)

Local tangent plane coordinates are called ENU coordinates. "E" stands for East, "N" for North, and "U" for Up. To convert latitude and longitude to ENU, you also need to use the ECEF conversion. Additionally, for ENU, you need to decide in advance where the origin of the Earth's plane is and keep it in ECEF coordinates. The conversion formula can be written in a programming language style as follows:

- `Xe`, `Ye`, `Ze`: ECEF coordinates of the point you want to convert
- `x0`, `y0`, `z0`: ECEF coordinates of the point you want to set as the origin of the local tangent plane
- `Lat`, `Lon`: Latitude and longitude of the point you want to convert

```plaintext
E = -sin(Lon * PI / 180) * (Xe - x0) + cos(Lon * PI / 180) * (Ye - y0);
N = -sin(Lat * PI / 180) * cos(Lon * PI / 180) * (Xe - x0) - sin(Lat * PI / 180) * sin(Lon * PI / 180) * (Ye - y0) + cos(Lat * PI / 180) * (Ze - z0);
U = cos(Lat * PI / 180) * cos(Lon * PI / 180) * (Xe - x0) + cos(Lat) * sin(Lon * PI / 180) * (Ye - y0) + sin(Lat * PI / 180) * (Ze - z0);
```

https://memo--randum.blogspot.com/2010/06/gps.html

# Estimate Yaw Angle

GPSデータから移動方向（ヨー角）を推定するためには、2点間のENU座標系における座標値を使用して、以下の手順で計算します。

1. **2点間のENU座標を取得**: 2点間の位置をGPSデータから取得し、それぞれの点をECEF座標系に変換した後、ENU座標系に変換します。これにより、2点間の東西方向の変位（ΔE）、南北方向の変位（ΔN）を得ます。

2. **ヨー角の計算**: ヨー角（θ）は、東向きを0度として、北向きに向かって正の角度（反時計回り）で表されます。この角度は、次の式で計算できます。

   $$
   \theta = \arctan2(\Delta N, \Delta E)
   $$

   ここで、$\arctan2(\Delta N, \Delta E)$ は、2点間の北向き変位と東向き変位から角度を計算する関数です。この関数は、-πからπの範囲で角度を返します。通常、結果を0度から360度の範囲に変換するために、負の値に360度を加えることがあります。

3. **角度の変換** (オプション): 必要に応じて、計算されたヨー角を他の単位や表記法に変換します。

これにより、GPSデータから2点間の移動方向（ヨー角）を推定することができます。

```cpp
#include <iostream>
#include <cmath>

// ラジアンから度への変換関数
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 2点間のヨー角を計算する関数
double calculateYawAngle(double E1, double N1, double E2, double N2) {
    double deltaE = E2 - E1; // 東西方向の変位
    double deltaN = N2 - N1; // 南北方向の変位
    double yawAngleRad = std::atan2(deltaN, deltaE); // ラジアンでのヨー角
    double yawAngleDeg = rad2deg(yawAngleRad);       // 度でのヨー角

    // 角度を0～360度の範囲に変換
    if (yawAngleDeg < 0) {
        yawAngleDeg += 360.0;
    }

    return yawAngleDeg;
}

int main() {
    // 例: 2点間のENU座標値
    double E1 = 100.0, N1 = 200.0; // 1点目のENU座標
    double E2 = 110.0, N2 = 205.0; // 2点目のENU座標

    // ヨー角の計算
    double yawAngle = calculateYawAngle(E1, N1, E2, N2);

    // 結果の出力
    std::cout << "Yaw Angle: " << yawAngle << " degrees" << std::endl;

    return 0;
}
```

## FlowChart

```mermaid
flowchart TD
    A[Start] --> B[Initialize ROS Node: gnss_preprocessing]
    B --> C[Create GnssPreprocessingCore Object with Initial GPS Coordinates]
    C --> D[Set Up ROS Publishers and Subscriber]
    D --> E[Main ROS Loop]
    E --> F[Check if ROS is OK]
    F -->|Yes| G[Wait for GNSS Data Callback]
    G --> E
    F -->|No| H[End]

    subgraph GnssPreprocessingCore
        I[Constructor: Initialize Variables and Set Up ROS Publishers and Subscriber]
        J[gnssCallback: Process GNSS Data]
        K[Convert Lat/Lon/Alt to ECEF Coordinates]
        L[Convert ECEF to ENU Coordinates]
        M[Publish GNSS Pose and Path]
        N[Transform Broadcast: /odom to /base_link]
    end

    I --> J
    J --> K
    K --> L
    L --> M
    M --> N

```

## State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> Uninitialized
    Uninitialized --> Initialized: Constructor
    Initialized --> ProcessingGNSSData: gnssCallback()
    ProcessingGNSSData --> Initialized: After Callback Processing
    ProcessingGNSSData --> [*]: Destructor
    Initialized --> [*]: Destructor
```
