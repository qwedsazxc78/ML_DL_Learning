# coding: utf-8
class Car_Detection:

    def __init__(self, MagX, MagY, MagZ):
        self.MagX_init = MagX
        self.MagY_init = MagY
        self.MagZ_init = MagZ
        self.MagLen_init = (MagX**2 + MagY ** 2 + MagZ**2)**(0.5)
        self.cos_alpha_init = self.MagX_init / self.MagLen_init
        self.cos_beta_init = self.MagY_init / self.MagLen_init
        self.cos_gamma_init = self.MagZ_init / self.MagLen_init
        self.index_performance_XY_plane = 0

    def get_mag_length(self, MagX, MagY, MagZ):
        return (MagX**2 + MagY ** 2 + MagZ**2)**(0.5)

    # "Analysis of Magnetic Field Disturbance Curve for Vehicle Presence Detection"
    def detect_1_Zcompoment_norm(self, MagX, MagY, MagZ):
        Z_compoment = 1000 * (MagZ / self.MagZ_init - 1)
        return Z_compoment

    def detect_2_performance_XY_plane(self, MagX, MagY, MagZ):
        P = (MagX - MagY) / (MagX**2 + MagY ** 2)**(0.5)
        return P

    def detect_3_cosine_variations_product_XY_plane(self, MagX, MagY, MagZ):
        MagLen = self.get_mag_length(MagX, MagY, MagZ)
        X_part = MagX / MagLen - self.MagX_init / self.MagLen_init
        Y_part = MagY / MagLen - self.MagY_init / self.MagLen_init
        CP = X_part / Y_part
        return CP

    def detect_4_cosine_variations_Kcriterion(self, MagX, MagY, MagZ):
        MagLen = self.get_mag_length(MagX, MagY, MagZ)
        X_part = MagX / MagLen - self.MagX_init / self.MagLen_init
        Y_part = MagY / MagLen - self.MagY_init / self.MagLen_init
        Z_part = MagZ / self.MagZ_init - 1
        K = 100 * (abs(X_part) + abs(Y_part) + Z_part)
        return K

    # "Vehicle Influence on the Earth’s Magnetic Field Changes"
    def detect_5_square_deviation(self, MagX, MagY, MagZ):
        K = self.get_mag_length(MagX - self.MagX_init,
                                MagY - self.MagY_init, MagZ - self.MagZ_init)
        return K

    def detect_6_vectorial_deviation(self, MagX, MagY, MagZ):
        MagLen = self.get_mag_length(MagX, MagY, MagZ)
        cos_alpha = MagX / MagLen
        cos_beta = MagY / MagLen
        cos_gamma = MagZ / MagLen
        K = abs(cos_alpha - self.cos_alpha_init) + abs(cos_beta -
                                                       self.cos_beta_init) + abs(cos_gamma - self.cos_gamma_init)
        return K

    def detect_7_combined_vectorial_deviation(self, MagX, MagY, MagZ):
        MagLen = self.get_mag_length(MagX, MagY, MagZ)
        cos_alpha = MagX / MagLen
        cos_beta = MagY / MagLen
        K = abs(cos_alpha - self.cos_alpha_init) + abs(cos_beta -
                                                       self.cos_beta_init) + (MagZ / self.MagZ_init - 1)
        return K

    def detect_8_weighting_vectorial_deviation(self, MagX, MagY, MagZ):
        Kd = 20  # mG
        Z_compoment = MagZ / self.MagZ_init - 1
        # if Z_compoment > -Kd, weight = 1
        # if Z_compoment < -Kd, weight = -1
        if Z_compoment > -Kd:
            weight = 1
        else:
            weight = -1

        MagLen = self.get_mag_length(MagX, MagY, MagZ)
        cos_alpha = MagX / MagLen
        cos_beta = MagY / MagLen

        K = abs(cos_alpha - self.cos_alpha_init) + \
            abs(cos_beta - self.cos_beta_init) + weight * Z_compoment
        return K

    def detect_9_each_axis_difference(self, MagX, MagY, MagZ):
        MagX_offset = MagX - self.MagX_init
        MagY_offset = MagY - self.MagY_init
        MagZ_offset = MagZ - self.MagZ_init

        K = abs(MagX_offset - MagY_offset) + abs(MagX_offset -
                                                 MagZ_offset) + abs(MagY_offset - MagZ_offset)
        return K

    def detect_10_each_axis_distance(self, MagX, MagY, MagZ):
        MagX_offset = MagX - self.MagX_init
        MagY_offset = MagY - self.MagY_init
        MagZ_offset = MagZ - self.MagZ_init

        K = abs(MagX_offset) + abs(MagY_offset) + abs(MagZ_offset)
        return K
