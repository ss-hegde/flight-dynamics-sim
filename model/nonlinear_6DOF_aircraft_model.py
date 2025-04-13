import numpy as np
DEBUG = False

def aircraft_dynamics(X, U, params):
    """
    Nonlinear 6-DOF aircraft model 

    States:
        X = [u, v, w, p, q, r, PHI, THETA, PSI, lambda, phi, h]
    Inputs:
        U = [d_eta, d_xi, d_zeta, d_T]  (elevator, aileron, rudder, throttle)
    """

    # Unpack state
    x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12 = X
    u1, u2, u3, u4, u5 = U

    # Unpack geometric parameters
    m = params['mass']
    g = params['gravity']
    Ixx = params['I_XX']
    Iyy = params['I_YY']
    Izz = params['I_ZZ']
    Ixy = params['I_XY']
    Ixz = params['I_XZ']
    Iyz = params['I_YZ']
    Iyx = params['I_YX']
    Izx = params['I_ZY']
    Izy = params['I_ZX']
    Inv_Ixx = params['Inv_I_XX']
    Inv_Iyy = params['Inv_I_YY']
    Inv_Izz = params['Inv_I_ZZ']
    Inv_Ixy = params['Inv_I_XY']
    Inv_Ixz = params['Inv_I_XZ']
    Inv_Iyx = params['Inv_I_YX']
    Inv_Iyz = params['Inv_I_YZ']
    Inv_Izx = params['Inv_I_ZX']
    Inv_Izy = params['Inv_I_ZY']
    S = params['S']
    St = params['St']
    lt = params['lt']
    cbar = params['cbar']
    X_G = params['X_G']
    Y_G = params['Y_G']
    Z_G = params['Z_G']
    X_ac = params['X_ac']
    Y_ac = params['Y_ac']
    Z_ac = params['Z_ac']
    X_A_P_T1 = params['X_A_P_T1']
    X_A_P_T2 = params['X_A_P_T2']
    Y_A_P_T1 = params['Y_A_P_T1']
    Y_A_P_T2 = params['Y_A_P_T2']
    Z_A_P_T1 = params['Z_A_P_T1']
    Z_A_P_T2 = params['Z_A_P_T2']
    rho = params['rho']

    I_G_BB = m* np.array([[Ixx, Ixy, Ixz],
                          [Ixy, Iyy, Iyz],
                          [Ixz, Iyz, Izz]])
    
    INV_I_G_BB = (1/m) * np.array([[Inv_Ixx, Inv_Ixy, Inv_Ixz],
                                   [Inv_Ixy, Inv_Iyy, Inv_Iyz],
                                   [Inv_Ixz, Inv_Iyz, Inv_Izz]])
    
    ######## VARIABLES ########

    # Calculate the airspeed and angles
    VEL_K_G_B_E_abs = np.sqrt(x1**2 + x2**2 + x3**2)

    # Calculate the angles
    if VEL_K_G_B_E_abs != 0:
        alpha_K_B = np.arctan2(x3, x1)  # Angle of attack
        beta_K_B = np.arcsin(x2 / VEL_K_G_B_E_abs)  # Sideslip angle
        # Removed unused variable: gamma_K_B
    else:
        alpha_K_B = 0.0
        beta_K_B = 0.0
        # Removed unused variable: gamma_K_B

    # Calculate dynamic pressure
    q_bar = 0.5 * rho * VEL_K_G_B_E_abs**2

    # Vectors
    OMEGA_K_OB_B = np.array([x4, x5, x6])  # Angular velocity vector in body frame (rad/s)
    VEL_K_G_B_E = np.array([x1, x2, x3])  # Velocity vector in body frame (m/s)

    #######################################################################
    ## AERODYNAMICS
    #######################################################################

    # Aerodynamic constants
    deps_dalpha = 0.25 # change in downwash angle per radian of alpha
    alpha_0 = -11.5*np.pi/180 # angle of attack at zero lift (radians)
    n = 5.5 # lift curve slope (1/rad)
    alpha3 = -768.5 # Coefficient of alpha^3 
    alpha2 = 609.2 # Coefficient of alpha^2
    alpha1 = -155.2 # Coefficient of alpha
    alpha0 = 12.5 # Coefficient of alpha^0
    alpha_switch = 14.5*(np.pi/180) # angle of attack at which to switch from polynomial to linear model (radians)

    # Calculate Cl_wb
    if alpha_K_B < alpha_switch:
        CL_wb = n*(alpha_K_B - alpha_0) 
    else:
        CL_wb = alpha3*alpha_K_B**3 + alpha2*alpha_K_B**2 + alpha1*alpha_K_B + alpha0

    # Calculate CL_t
    epsilon = deps_dalpha * (alpha_K_B - alpha_0) # downwash angle (radians)
    alpha_t = alpha_K_B - epsilon + u2 + (1.3*x5*lt/VEL_K_G_B_E_abs) # angle of attack at the tail (radians)
    CL_t = 3.1 * (St/S) * alpha_t # lift coefficient at the tail (dimensionless)
    
    # Total lift coefficient
    CL = CL_wb + CL_t # total lift coefficient (dimensionless)

    # Total drag coefficient
    CD = 0.13 + 0.07* (5.5*alpha_K_B + 0.654)**2

    # Total side force coefficient
    CY = -1.6*beta_K_B + 0.243*u3

    #############################################################################
    ## AERODYNAMIC FORCES

    F_A_A_A = np.array([
                        -CD*q_bar*S,    # Drag force (x-direction)
                        CY*q_bar*S,     # Side force (y-direction)
                        -CL*q_bar*S     # Lift force (z-direction)
                        ])
    
    # Rotation of aerodynamic frame to body frame

    M_BA = np.array([[np.cos(alpha_K_B), 0, -np.sin(alpha_K_B)],
                     [0, 1, 0],
                     [np.sin(alpha_K_B), 0, np.cos(alpha_K_B)]]) * \
           np.array([[np.cos(beta_K_B), -np.sin(beta_K_B), 0],
                     [np.sin(beta_K_B), np.cos(beta_K_B), 0],
                     [0, 0, 1]])
    
    F_A_G_B = np.dot(M_BA, F_A_A_A) # Aerodynamic forces in body frame (N)

    # Moments due to aerodynamic forces
    eta11 = 1.4 * beta_K_B
    eta21 = -0.59 - (3.1 * (St*lt)/(S*cbar))*(alpha_K_B - epsilon)
    eta31 = (1 - alpha_K_B*(180/(15*np.pi)))* beta_K_B

    eta = np.array([eta11, eta21, eta31])

    dCMdx = (cbar/VEL_K_G_B_E_abs) * np.array([[-11, 0, 5],
                                               [0, (-4.03*(St*lt**2)/(S*cbar**2)), 0],
                                               [1.7, 0, -11.5]])
    
    dCMdu = np.array([[-0.6, 0, 0.22],
                      [0, (-3.1*(St*lt)/(S*cbar)), 0],
                      [0, 0, -0.64]])
    
    # Calculate CM = np.array([Cl, Cm, Cn]) about the aerodynamic center in body frame
    CM_ac_B = eta + np.dot(dCMdx, OMEGA_K_OB_B) + np.dot(dCMdu, np.array([u1, u2, u3]))

    # Aerodynamic moment
    MA_ac_B = CM_ac_B * q_bar * S * cbar

    # Transfer the aerodynamic forces and moments to the center of gravity
    r_G_B = np.array([X_G, Y_G, Z_G])
    r_ac_B = np.array([X_ac, Y_ac, Z_ac])
    M_A_G_B = MA_ac_B + np.cross(F_A_G_B, r_G_B - r_ac_B) # Aerodynamic moment about the center of gravity (N*m)

    #############################################################################
    ## PROPULSION FORCES
    #############################################################################

    F1 = u4 * m * g
    F2 = u5 * m * g

    F_P_T1_G_B = np.array([F1, 0, 0]) # Propulsion force in body frame (N)
    F_P_T2_G_B = np.array([0, F2, 0]) # Propulsion force in body frame (N)

    # Total propulsion force
    F_P_G_B = F_P_T1_G_B + F_P_T2_G_B

    # Engine moment due to the offset of the thrust vector at the center of gravity

    r_P_T1_G_B = np.array([X_G - X_A_P_T1, Y_A_P_T1-Y_G, Z_G - Z_A_P_T1])
    r_P_T2_G_B = np.array([X_G - X_A_P_T2, Y_A_P_T2-Y_G, Z_G - Z_A_P_T2])

    M_P1_G_B = np.cross(r_P_T1_G_B, F_P_T1_G_B) # Engine moment about the center of gravity (N*m)
    M_P2_G_B = np.cross(r_P_T2_G_B, F_P_T2_G_B) # Engine moment about the center of gravity (N*m)

    M_P_G_B = M_P1_G_B + M_P2_G_B # Total engine moment about the center of gravity (N*m)
    
    #############################################################################
    ## GRAVITY FORCES
    #############################################################################

    g_B = np.array([-g*np.sin(x8), g*np.cos(x8)*np.sin(x7), g*np.cos(x8)*np.cos(x7)]) # Gravity vector in body frame (N)

    F_G_G_B = m * g_B # Gravity force in body frame (N)

    ################################################################################
    # STATE DERIVATIVES
    ################################################################################
    # Translational dynamics

    F_G_B = F_G_G_B + F_A_G_B + F_P_G_B # Total force in body frame (N)
    VEL_DOT_K_G_EB = (1/m) * F_G_B - np.cross(OMEGA_K_OB_B, VEL_K_G_B_E) # Translational acceleration in body frame (m/s^2)
    
    if DEBUG:
        print("VEL_DOT_K_G_EB", VEL_DOT_K_G_EB.shape)

    # Rotational dynamics

    M_G_B = M_A_G_B + M_P_G_B 
    OMEGA_DOT_K_OB_B_B = np.dot(INV_I_G_BB, M_G_B - np.cross(OMEGA_K_OB_B, np.dot(I_G_BB, OMEGA_K_OB_B))) # Rotational acceleration in body frame (rad/s^2)
    
    if DEBUG:
        print("OMEGA_DOT_K_OB_B_B", OMEGA_DOT_K_OB_B_B.shape)


    # Calculate PHI_DOT, THETA_DOT, PSI_DOT (ATTITUDE)
    M_phi = np.array([[1, np.sin(x7)*np.tan(x8), np.cos(x7)*np.tan(x8)],
                        [0, np.cos(x7), -np.sin(x7)], 
                        [0, np.sin(x7)/np.cos(x8), np.cos(x7)/np.cos(x8)]])
    
    ATT_DOT_K_OB_B = np.dot(M_phi, OMEGA_K_OB_B) # Attitude rate in body frame (rad/s)
    
    if DEBUG:
        print("ATT_DOT_K_OB_B", ATT_DOT_K_OB_B.shape)

    # Calculate lambda_dot, phi_dot, h_dot (POSITION)

    t1 = np.array([[1, 0, 0],
                    [0, np.cos(x7), np.sin(x7)],
                    [0, -np.sin(x7), np.cos(x7)]])
    
    t2 = np.array([[np.cos(x8), 0, -np.sin(x8)],
                    [0, 1, 0],
                    [np.sin(x8), 0, np.cos(x8)]])
    
    t3 = np.array([[np.cos(x9), np.sin(x9), 0],
                    [-np.sin(x9), np.cos(x9), 0],
                    [0, 0, 1]])
    
    M_BO = np.dot(t1, np.dot(t2, t3)) # Body to Earth frame transformation matrix

    M_OB = np.transpose(M_BO) # Earth to body frame transformation matrix

    VEL_K_G_O_E = np.dot(M_OB, VEL_K_G_B_E) # Velocity vector in Earth frame (m/s)  

    u_K_G_O_E = VEL_K_G_O_E[0] # Velocity in Earth frame (m/s)
    v_K_G_O_E = VEL_K_G_O_E[1] # Velocity in Earth frame (m/s)
    w_K_G_O_E = VEL_K_G_O_E[2] # Velocity in Earth frame (m/s)

    a = 6378137.0 # Radius of the Earth (m) (semi-major axis)
    b = 6356752.314245 # Radius of the Earth (m) (semi-minor axis)
    f = 1 - (b/a) # Flattening of the Earth
    e = np.sqrt(2*f - f**2) # Eccentricity of the Earth

    N_mu = a / np.sqrt(1 - e**2 * np.sin(x11)**2) # Radius of curvature in the prime vertical (m)
    M_mu = a * (1 - e**2) / (1 - e**2 * np.sin(x11)**2)**(3/2) # Radius of curvature in the meridian (m)

    POS_DOT_O_E = np.array([v_K_G_O_E/((N_mu + x12)*np.cos(x11)),
                            u_K_G_O_E/(M_mu + x12),
                            -w_K_G_O_E]) # Position rate in Earth frame (m/s)
    
    if DEBUG:
        print("POS_DOT_O_E", POS_DOT_O_E.shape)

    # Concatenate the translational, rotation, attitude, and position rates
    X_DOT = np.concatenate((VEL_DOT_K_G_EB, OMEGA_DOT_K_OB_B_B, ATT_DOT_K_OB_B, POS_DOT_O_E), axis=0) # State rate (m/s, rad/s, rad/s, m/s)

    return X_DOT

