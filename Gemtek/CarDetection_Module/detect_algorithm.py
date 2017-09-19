# read time
None_Status = 1
Env_Measure = 1
Long_Measure = 3
Smooth_Measure = 1
Quick_Measure = 0.5

IST_TRUE = 1
IST_FALSE = 0

Mag_LastStateEnable = 0

MagVehicleState_Moving = 2
MagVehicleState_Car = 1
MagVehicleState_Empty = 0
MagLSB2mG = 1.5

STABLE_THRESHOLD = 10  # mG
ACTIVE_THRESHOLD = 20  # mG
DETECTION_THRESHOLD = 50  # mG
CAR_STABLE_THRESHOLD_1 = 10  # 5s
CAR_STABLE_THRESHOLD_2 = 15  # 10s

STABLE_COUNTER_THRESHOLD = 20

Env_update_time = 3  # update 3s
Env_update_factor = 0.05
SendData_Stable_Counter = 10

Detect_Feature_Weight = 100 / 5


class detect_algorithm:

    def __init__(self):
        self.Env_Bias = [0, 0, 0]
        self.Env_update_index = 0
        self.Env_Calibrated_Status = IST_FALSE
        self.Calibrated_Data = [0, 0, 0]
        self.Data_Buffer_Index = 0
        self.Distorted_Vector = 0
        self.PreDistorted_Vector = 0
        self.Mag_stable_index = 0
        self.CarDetection_status = MagVehicleState_Empty
        self.Measure_Rate = Env_Measure
        self.SendData_status = IST_TRUE
        self.SendData_Car_Index = 0
        self.SendData_Empty_Index = 0
        self.Detect_Feature = [0, 0, 0, 0]
        self.Detect_Score = 0

    def Get_VehicleDetection_Status(self):
        return self.CarDetection_status

    def Get_VehicleDetection_SendData(self):
        return self.SendData_status

    def Get_VehicleDetection_Rate(self, MagRawData):
        return self.VehicleDetection(MagRawData)

    def Get_VehicleDetection_Score(self):
        return self.Detect_Score

    def Set_VehicleDetection_EnvBias(self, set_bias):
        self.Env_Bias[0] = set_bias[0] * MagLSB2mG
        self.Env_Bias[1] = set_bias[1] * MagLSB2mG
        self.Env_Bias[2] = set_bias[2] * MagLSB2mG

    def Get_VehicleDetection_EnvBias(self):
        bias = [0, 0, 0]
        bias[0] = self.Env_Bias[0] / MagLSB2mG
        bias[1] = self.Env_Bias[1] / MagLSB2mG
        bias[2] = self.Env_Bias[2] / MagLSB2mG
        return bias

    def Env_Calibration(self, MagRawData):
        if (self.Env_update_index == 0):
            for i in range(0, 3):
                self.Env_Bias[i] = MagRawData[i] * MagLSB2mG
        else:
            for i in range(0, 3):
                self.Env_Bias[i] = (
                    MagRawData[i] * MagLSB2mG + self.Env_Bias[i]) / 2

        self.Env_update_index += 1

        if (self.Env_update_index >= Env_update_time):
            self.Env_Calibrated_Status = IST_TRUE
            self.Measure_Rate = Long_Measure
            self.Mag_stable_index = STABLE_COUNTER_THRESHOLD

        else:
            self.Env_Calibrated_Status = IST_FALSE
            self.Measure_Rate = Env_Measure

    def Env_Adaptive_Offset(self, MagRawData):
        for i in range(0, 3):
            self.Env_Bias[i] = Env_update_factor * MagRawData[i] * \
                MagLSB2mG + (1 - Env_update_factor) * self.Env_Bias[i]

    def VehicleDetection_SendData(self):
        if (self.CarDetection_status == MagVehicleState_Empty and
                self.Measure_Rate == Long_Measure):
            self.SendData_Empty_Index += 1
            self.SendData_Car_Index = 0
        elif (self.CarDetection_status == MagVehicleState_Car and
              self.Measure_Rate == Long_Measure):
            self.SendData_Car_Index += 1
            self.SendData_Empty_Index = 0
        else:
            self.SendData_Car_Index = 0
            self.SendData_Empty_Index = 0

        if (self.SendData_Empty_Index > STABLE_COUNTER_THRESHOLD):
            self.SendData_Empty_Index = STABLE_COUNTER_THRESHOLD

        if (self.SendData_Car_Index > STABLE_COUNTER_THRESHOLD):
            self.SendData_Car_Index = STABLE_COUNTER_THRESHOLD

        # stable 30s, stop send data for power consumption
        if (self.SendData_Empty_Index >= SendData_Stable_Counter or
                self.SendData_Car_Index >= SendData_Stable_Counter):
            self.SendData_status = IST_FALSE  # don't send data
        else:
            self.SendData_status = IST_TRUE  # send data

    def Get_VectorNorm(self, Data, DataSize):
        Length = 0
        for i in range(0, DataSize):
            Length += Data[i] * Data[i]
        return Length**(0.5)

    def VehicleDetection(self, MagRawData):
        if (self.Env_Calibrated_Status == IST_FALSE):
            self.Env_Calibration(MagRawData)
            return self.Measure_Rate

        for i in range(0, 3):
            self.Calibrated_Data[i] = (
                MagRawData[i] * MagLSB2mG - self.Env_Bias[i])

        self.PreDistorted_Vector = self.Distorted_Vector
        self.Distorted_Vector = self.Get_VectorNorm(self.Calibrated_Data, 3)

        # check if mag data sequence is stable
        if ((self.PreDistorted_Vector - self.Distorted_Vector) < STABLE_THRESHOLD and
                (self.PreDistorted_Vector - self.Distorted_Vector) > -STABLE_THRESHOLD):
            if (self.Measure_Rate == Long_Measure):
                self.Mag_stable_index += 3
            else:
                self.Mag_stable_index += 1

        else:
            if (self.Measure_Rate == Long_Measure):
                self.Mag_stable_index -= 3
            else:
                self.Mag_stable_index -= 1

        if (self.Mag_stable_index > 20):
            self.Mag_stable_index = 20

        elif (self.Mag_stable_index < 0):
            self.Mag_stable_index = 0

        # check if need to send more data
        self.VehicleDetection_SendData()

        self.Detect_Feature[0] = self.detect_feature_each_axis_difference(
            self.Calibrated_Data)
        self.Detect_Feature[1] = self.detect_feature_each_axis_distance(
            self.Calibrated_Data)
        self.Detect_Feature[2] = self.detect_feature_square_deviation(
            self.Calibrated_Data)
        self.Detect_Feature[
            3] = self.detect_feature_vectorial_deviation(MagRawData)
        self.Detect_Feature[
            4] = self.detect_feature_weighting_vectorial_deviation(MagRawData)

        self.Detect_Score = 0
        for i in range(0, len(self.Detect_Feature)):
            if self.Detect_Feature[i] > DETECTION_THRESHOLD:
                self.Detect_Score += Detect_Feature_Weight

        # check status
        if (self.Distorted_Vector <= ACTIVE_THRESHOLD):
            # if car go out quickly, and Distorted_Vector is closer to env value,
            # reset stable index
            if (self.CarDetection_status == MagVehicleState_Car):
                self.Mag_stable_index = 0

            if (self.Mag_stable_index <= CAR_STABLE_THRESHOLD_1):
                # IST_DEBUG_PRINT("[D]: A1   ");
                self.CarDetection_status = MagVehicleState_Moving
                self.Measure_Rate = Quick_Measure
                return Quick_Measure

            elif (self.Mag_stable_index > CAR_STABLE_THRESHOLD_1 and
                  self.Mag_stable_index <= CAR_STABLE_THRESHOLD_2):
                # IST_DEBUG_PRINT("[D]: A2   ");
                self.CarDetection_status = MagVehicleState_Empty
                self.Measure_Rate = Smooth_Measure
                return Smooth_Measure

            else:
                # IST_DEBUG_PRINT("[D]: A3   ");
                self.CarDetection_status = MagVehicleState_Empty
                self.Measure_Rate = Long_Measure
                self.Env_Adaptive_Offset(MagRawData)
                return Long_Measure

            # self.CarDetection_status = MagVehicleState_Empty;
            # return Long_Measure;

        # need to add a mechnism to skip stay this status after long time
        elif (self.Distorted_Vector > ACTIVE_THRESHOLD and self.Distorted_Vector <= DETECTION_THRESHOLD):
            # IST_DEBUG_PRINT("[D]: B1   ");
            self.CarDetection_status = MagVehicleState_Moving
            self.Measure_Rate = Smooth_Measure
            self.set_detect_feature_car()

            return Smooth_Measure

        else:  # car in state
            if (self.CarDetection_status == MagVehicleState_Empty):
                self.Mag_stable_index = 0

            if (self.Mag_stable_index <= CAR_STABLE_THRESHOLD_1):
                # IST_DEBUG_PRINT("[D]: C1   ");
                self.CarDetection_status = MagVehicleState_Moving
                self.Measure_Rate = Quick_Measure
                self.set_detect_feature_car()
                return Quick_Measure

            elif (self.Mag_stable_index > CAR_STABLE_THRESHOLD_1 and
                  self.Mag_stable_index <= CAR_STABLE_THRESHOLD_2):
                # IST_DEBUG_PRINT("[D]: C2   ");
                self.CarDetection_status = MagVehicleState_Car
                self.Measure_Rate = Smooth_Measure
                return Smooth_Measure

            else:
                # IST_DEBUG_PRINT("[D]: C3   ");
                self.CarDetection_status = MagVehicleState_Car
                self.Measure_Rate = Long_Measure
                return Long_Measure

    def set_detect_feature_car(self):
        if self.Detect_Score == 100:
            self.CarDetection_status = MagVehicleState_Car

    def detect_feature_each_axis_difference(self, MagCaliData):
        K = abs(MagCaliData[0] - MagCaliData[1]) + \
            abs(MagCaliData[0] - MagCaliData[2]) + \
            abs(MagCaliData[1] - MagCaliData[2])
        return K

    def detect_feature_each_axis_distance(self, MagCaliData):
        K = abs(MagCaliData[0]) + abs(MagCaliData[1]) + abs(MagCaliData[2])
        return K

    # "Vehicle Influence on the Earth’s Magnetic Field Changes"
    def detect_feature_square_deviation(self, MagCaliData):
        K = self.Get_VectorNorm(MagCaliData, 3)
        return K

    def detect_feature_vectorial_deviation(self, MagRawData):
        MagLen = self.Get_VectorNorm(MagRawData, 3)
        cos_alpha = MagRawData[0] / MagLen
        cos_beta = MagRawData[1] / MagLen
        cos_gamma = MagRawData[2] / MagLen
        MageEnv_Len = self.Get_VectorNorm(self.Env_Bias, 3)

        cos_alpha_init = self.Env_Bias[0] / MageEnv_Len
        cos_beta_init = self.Env_Bias[1] / MageEnv_Len
        cos_gamma_init = self.Env_Bias[2] / MageEnv_Len
        K = abs(cos_alpha - cos_alpha_init) + abs(cos_beta -
                                                  cos_beta_init) + abs(cos_gamma - cos_gamma_init)
        return 1000 * K

    def detect_feature_weighting_vectorial_deviation(self, MagRawData):
        Kd = 20  # mG
        Z_compoment = MagRawData[2] / self.Env_Bias[2] - 1
        # if Z_compoment > -Kd, weight = 1
        # if Z_compoment < -Kd, weight = -1
        if Z_compoment > -Kd:
            weight = 1
        else:
            weight = -1

        MagLen = self.Get_VectorNorm(MagRawData, 3)
        cos_alpha = MagRawData[0] / MagLen
        cos_beta = MagRawData[1] / MagLen
        MageEnv_Len = self.Get_VectorNorm(self.Env_Bias, 3)
        cos_alpha_init = self.Env_Bias[0] / MageEnv_Len
        cos_beta_init = self.Env_Bias[1] / MageEnv_Len
        K = abs(cos_alpha - cos_alpha_init) + abs(cos_beta -
                                                  cos_beta_init) + weight * Z_compoment
        return K
