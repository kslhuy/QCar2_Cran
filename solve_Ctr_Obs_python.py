"""
Description:
This file is used to solve the observer gain of the distributed observer
for a platoon of vehicles. It uses LMI to find the observer gain

Converted from MATLAB to Python
"""

import numpy as np
from scipy.linalg import block_diag, eig
from scipy.signal import StateSpace
import cvxpy as cp

# Helper function to create block diagonal matrix N times
def makeBlockDiag(matrix, N):
    """Create a block diagonal matrix by repeating the input matrix N times"""
    matrices = [matrix for _ in range(N)]
    return block_diag(*matrices)

def makeBlockDiag_cvxpy(matrix, N):
    """Create a block diagonal matrix for CVXPY expressions by repeating N times"""
    import cvxpy as cp
    if N == 1:
        return matrix
    # Get dimensions
    rows, cols = matrix.shape
    # Create list of blocks for each row
    blocks = []
    for i in range(N):
        row_blocks = []
        for j in range(N):
            if i == j:
                row_blocks.append(matrix)
            else:
                row_blocks.append(np.zeros((rows, cols)))
        blocks.append(cp.hstack(row_blocks))
    return cp.vstack(blocks)

# Helper function to check observability
def is_observable(A, C):
    """Check if the system (A, C) is observable"""
    n = A.shape[0]
    O = C.copy()
    for i in range(1, n):
        O = np.vstack([O, C @ np.linalg.matrix_power(A, i)])
    return np.linalg.matrix_rank(O) == n

def main():
    # Define system parameters
    n0 = 3  # Number of states (position, velocity, acceleration)
    N = 3   # Number of vehicles in the platoon
    n = n0 * N  # Total number of states
    m = 2   # Number of outputs (position and velocity)
    q = 1   # Number of disturbances

    # Time constant and other parameters
    tau = 0.16  # Time constant
    h = 1       # Distance between vehicles
    d = 0.8
    c = 0.45

    # Define system matrices
    A_tau = np.array([[0, 1, 0],
                      [0, 0, 1],
                      [0, 0, -1/tau]])
    
    B_tau = np.array([[0], [0], [1/tau]])
    
    A_h = np.array([[0, 0, h],
                    [0, 0, 0],
                    [0, 0, 0]])
    
    B_delta = block_diag(B_tau, B_tau, B_tau)
    
    A_h_tau = A_h + A_tau
    
    A_delta = np.block([
        [A_h_tau, np.zeros((n0, n0)), np.zeros((n0, n0))],
        [A_h, A_h_tau, np.zeros((n0, n0))],
        [A_h, A_h, A_h_tau]
    ])
    
    D1 = np.array([[10], [5], [2]])
    D = block_diag(D1, D1, D1)
    
    # Define output matrices
    Cf = np.array([[1, -h, 0],
                   [0, 1, 0]])
    
    Cp = np.array([[-1, 0, 0],
                   [0, 0, 0]])
    
    C1 = np.block([Cf, np.zeros((2, n0)), np.zeros((2, n0))])
    C2 = np.block([Cp, Cf, np.zeros((2, n0))])
    C3 = np.block([np.zeros((2, n0)), Cp, Cf])
    
    Cv = np.array([[-h], [1]])
    Cd = np.array([[-1], [0]])
    
    # Define F matrices
    F1 = np.block([np.eye(n0), np.zeros((n0, n0)), np.zeros((n0, n0))])
    F2 = np.block([np.zeros((n0, n0)), np.eye(n0), np.zeros((n0, n0))])
    F3 = np.block([np.zeros((n0, n0)), np.zeros((n0, n0)), np.eye(n0)])
    
    # Communication topology adjacency matrix
    a = np.array([[0, 1, 0],
                  [1, 0, 1],
                  [0, 1, 0]])
    
    # Define bar_F matrices
    barF1 = np.vstack([F1, np.zeros((n0, n)), np.zeros((n0, n))])
    barF2 = np.vstack([F2, F2 - F1, np.zeros((n0, n))])
    barF3 = np.vstack([F3, F3 - F1, F3 - F2])
    
    F = np.hstack([barF1, barF2, barF3])
    calF = np.vstack([barF1, barF2, barF3])
    
    # Check observability
    C = np.vstack([C1, C2, C3])
    if is_observable(A_delta, C):
        print('The collective system is jointly observable')
    else:
        raise ValueError('The collective system is not observable')
    
    # Communication topology - Laplacian matrix
    Lap = np.array([[1, -1, 0],
                    [-1, 2, -1],
                    [0, -1, 1]])
    
    r = np.array([1, 1, 1])  # Example solution vector for testing
    R = np.diag(r)
    hatLap = R @ Lap + Lap.T @ R
    lamda = np.linalg.eigvals(hatLap)
    lamda = np.sort(lamda)
    lamda2 = lamda[1]
    
    iota1 = 0.5
    iota2 = 0.5
    alpha = 0.5
    
    print(f"\nSolving LMI with parameters:")
    print(f"N = {N}, n = {n}, m = {m}")
    print(f"tau = {tau}, h = {h}")
    print(f"lambda_2 = {lamda2:.4f}")
    
    # Define CVXPY variables
    mu = cp.Variable(nonneg=True)
    
    # Lyapunov matrices for distributed observer
    P1 = cp.Variable((n, n), symmetric=True)
    P2 = cp.Variable((n, n), symmetric=True)
    P3 = cp.Variable((n, n), symmetric=True)
    
    # Observer gains
    X1 = cp.Variable((n, m))
    X2 = cp.Variable((n, m))
    X3 = cp.Variable((n, m))
    
    gamma = cp.Variable(nonneg=True)
    
    # Lyapunov matrices for controller
    Q1 = cp.Variable((n0, n0), symmetric=True)
    Q2 = cp.Variable((n0, n0), symmetric=True)
    Q3 = cp.Variable((n0, n0), symmetric=True)
    
    # Controller gains
    Y11 = cp.Variable((1, n0))
    Y22 = cp.Variable((1, n0))
    Y21 = cp.Variable((1, n0))
    Y33 = cp.Variable((1, n0))
    Y31 = cp.Variable((1, n0))
    Y32 = cp.Variable((1, n0))
    
    # Create Y matrices with explicit zero padding
    zeros_1_n0 = np.zeros((1, n0))
    Y1 = cp.hstack([Y11, zeros_1_n0, zeros_1_n0])
    Y2 = cp.hstack([-Y21, Y22, zeros_1_n0])
    Y3 = cp.hstack([-Y31, -Y32, Y33])
    
    Y = cp.vstack([Y1, Y2, Y3])
    
    # Create block diagonal calY
    zeros_1_n = np.zeros((1, n))
    calY_row1 = cp.hstack([Y1, zeros_1_n, zeros_1_n])
    calY_row2 = cp.hstack([zeros_1_n, Y2, zeros_1_n])
    calY_row3 = cp.hstack([zeros_1_n, zeros_1_n, Y3])
    calY = cp.vstack([calY_row1, calY_row2, calY_row3])
    
    # Distributed observer design using LMI
    Lambda1 = A_delta.T @ P1 + P1 @ A_delta - C1.T @ X1.T - X1 @ C1
    Lambda2 = A_delta.T @ P2 + P2 @ A_delta - C2.T @ X2.T - X2 @ C2
    Lambda3 = A_delta.T @ P3 + P3 @ A_delta - C3.T @ X3.T - X3 @ C3
    
    Lambda_p = cp.hstack([Lambda1 + np.eye(n), Lambda2 + np.eye(n), Lambda3 + np.eye(n)])
    
    # Create block diagonal Lambda
    zeros_n_n = np.zeros((n, n))
    Lambda_row1 = cp.hstack([Lambda1, zeros_n_n, zeros_n_n])
    Lambda_row2 = cp.hstack([zeros_n_n, Lambda2, zeros_n_n])
    Lambda_row3 = cp.hstack([zeros_n_n, zeros_n_n, Lambda3])
    Lambda = cp.vstack([Lambda_row1, Lambda_row2, Lambda_row3])
    
    Gamma_p = cp.hstack([D.T @ P1, D.T @ P2, D.T @ P3])
    
    # M_c = sum Lambda_i + N*I_n
    Mc = Lambda1 + Lambda2 + Lambda3 + N * np.eye(n)
    
    # M_d = Lambda + I_nN - 2*gamma*lambda_2(hatL)*I_nN
    Md = Lambda + np.eye(n*N) - 2*gamma*lamda2*np.eye(n*N)
    dim_Md = n*N
    
    # Controller design - need to construct Q block diagonal for Me
    Q_blk = cp.bmat([[Q1, np.zeros((n0, n0)), np.zeros((n0, n0))],
                     [np.zeros((n0, n0)), Q2, np.zeros((n0, n0))],
                     [np.zeros((n0, n0)), np.zeros((n0, n0)), Q3]])
    
    Me = A_delta @ Q_blk + B_delta @ Y + Q_blk @ A_delta.T + Y.T @ B_delta.T
    
    bb_Me = cp.bmat([[Me, Q_blk],
                     [Q_blk.T, -np.eye(n)]])
    dim_bb_Me = bb_Me.shape[0]
    
    bb_Mc = cp.bmat([[Mc, -(P1 @ D + P2 @ D + P3 @ D)],
                     [-(P1 @ D + P2 @ D + P3 @ D).T, -mu * np.eye(N*q)]])
    dim_bb_Mc = bb_Mc.shape[0]
    
    # Define block matrices
    bb_D = np.block([[np.zeros((n, n)), D],
                     [np.zeros((n, n)), np.zeros((n, N*q))]])
    
    bb_Y1 = cp.bmat([[B_delta @ Y, B_delta @ calY],
                     [np.zeros((n, n)), np.zeros((n, n*N))]])
    
    bb_P_top = cp.hstack([Lambda_p.T, -Gamma_p.T])
    dim1 = n*(N**2 + N)
    bb_P = cp.vstack([
        bb_P_top,
        cp.hstack([np.zeros((dim1, n)), np.zeros((dim1, q*N))]),
        cp.hstack([np.zeros((dim1, n)), np.zeros((dim1, q*N))]),
        cp.hstack([np.zeros((n*N, n)), np.zeros((n*N, q*N))]),
        cp.hstack([np.zeros((n*N, n)), np.zeros((n*N, q*N))])
    ])
    
    bb_F1 = np.vstack([
        np.hstack([F.T, np.zeros((n*N, n*N))]),
        np.hstack([np.zeros((dim1, n)), np.zeros((dim1, n*N))]),
        np.hstack([np.zeros((dim1, n)), np.zeros((dim1, n*N))]),
        np.hstack([np.zeros((n*N, n)), np.zeros((n*N, n*N))]),
        np.hstack([np.zeros((n*N, n)), np.zeros((n*N, n*N))])
    ])
    
    bb_F2 = np.vstack([
        np.hstack([np.zeros((n, n)), calF.T]),
        np.hstack([np.zeros((N*q, n)), np.zeros((N*q, n*N))])
    ])
    
    # bb_F3
    ones_N = np.ones((N, 1))
    bb_F3 = np.hstack([
        makeBlockDiag(calF.T, N),
        -np.kron(ones_N, F).T
    ])
    
    # bb_Y2 - construct using CVXPY-aware block diagonal
    B_calY = B_delta @ calY  # This is a CVXPY expression
    B_Y = B_delta @ Y  # This is a CVXPY expression
    
    bb_Y2 = cp.hstack([
        makeBlockDiag_cvxpy(B_calY, N),
        makeBlockDiag_cvxpy(B_Y, N)
    ])
    
    # Create P block diagonal for bb_Md
    P_blk_row1 = cp.hstack([P1, np.zeros((n, n)), np.zeros((n, n))])
    P_blk_row2 = cp.hstack([np.zeros((n, n)), P2, np.zeros((n, n))])
    P_blk_row3 = cp.hstack([np.zeros((n, n)), np.zeros((n, n)), P3])
    P_blk = cp.vstack([P_blk_row1, P_blk_row2, P_blk_row3])
    
    # bb_Md - Large block matrix
    # Create block diagonal Q matrices for bb_Md
    Q_blk_diag_large = makeBlockDiag_cvxpy(Q_blk, N**2+N)
    
    bb_Md = cp.bmat([
        [Md, np.zeros((n*N, dim1)), bb_F3, P_blk, np.zeros((n*N, n*N))],
        [np.zeros((dim1, n*N)), -1/iota1 * Q_blk_diag_large, np.zeros((dim1, dim1)), np.zeros((dim1, n*N)), bb_Y2.T],
        [bb_F3.T, np.zeros((dim1, dim1)), -iota1 * Q_blk_diag_large, np.zeros((dim1, n*N)), np.zeros((dim1, n*N))],
        [P_blk, np.zeros((n*N, dim1)), np.zeros((n*N, dim1)), -1/iota2 * np.eye(n*N), np.zeros((n*N, n*N))],
        [np.zeros((n*N, n*N)), bb_Y2, np.zeros((n*N, dim1)), np.zeros((n*N, n*N)), -iota2 * np.eye(n*N)]
    ])
    dim_bb_Md = bb_Md.shape[0]
    
    # Final LMI matrix M
    dim_alpha = n * (N + 1)
    
    # Create block diagonal Q matrices for M
    Q_blk_diag_small = makeBlockDiag_cvxpy(Q_blk, N+1)
    
    M = cp.bmat([
        [bb_Me, np.zeros((dim_bb_Me, dim_bb_Md)), bb_D, bb_Y1, np.zeros((dim_bb_Me, dim_alpha))],
        [np.zeros((dim_bb_Md, dim_bb_Me)), bb_Md, bb_P, np.zeros((dim_bb_Md, dim_alpha)), bb_F1],
        [bb_D.T, bb_P.T, bb_Mc, np.zeros((dim_bb_Mc, dim_alpha)), bb_F2],
        [bb_Y1.T, np.zeros((dim_alpha, dim_bb_Md)), np.zeros((dim_alpha, dim_bb_Mc)), -1/alpha * Q_blk_diag_small, np.zeros((dim_alpha, dim_alpha))],
        [np.zeros((dim_alpha, dim_bb_Me)), bb_F1.T, bb_F2.T, np.zeros((dim_alpha, dim_alpha)), -alpha * Q_blk_diag_small]
    ])
    
    # Define constraints
    constraints = [
        M << 0,  # M is negative semidefinite
        bb_Md << 0,  # bb_Md is negative semidefinite
        P1 >> 0,  # P1 is positive definite
        P2 >> 0,  # P2 is positive definite
        P3 >> 0,  # P3 is positive definite
        Q1 >> 0,  # Q1 is positive definite
        Q2 >> 0,  # Q2 is positive definite
        Q3 >> 0,  # Q3 is positive definite
        mu >= 0,
        gamma >= 0
    ]
    
    # Define the problem (feasibility problem)
    prob = cp.Problem(cp.Minimize(0), constraints)
    
    # Solve the problem
    print("\nSolving LMI problem...")
    print("This may take several minutes...")
    
    try:
        prob.solve(solver=cp.SCS, verbose=True)
        # Alternative solvers you can try:
        # prob.solve(solver=cp.CVXOPT, verbose=True)
        # prob.solve(solver=cp.MOSEK, verbose=True)
    except Exception as e:
        print(f"Error during solving: {e}")
        print("Try installing additional solvers: pip install cvxopt mosek")
        return
    
    # Check if the problem is feasible
    if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print(f'\nLMI is feasible! (Status: {prob.status})')
        
        # Extract solutions
        P1_val = P1.value
        P2_val = P2.value
        P3_val = P3.value
        
        X1_val = X1.value
        X2_val = X2.value
        X3_val = X3.value
        
        # Calculate observer gains
        L1 = np.linalg.inv(P1_val) @ X1_val
        L2 = np.linalg.inv(P2_val) @ X2_val
        L3 = np.linalg.inv(P3_val) @ X3_val
        
        Q1_val = Q1.value
        Q2_val = Q2.value
        Q3_val = Q3.value
        
        Y11_val = Y11.value
        Y22_val = Y22.value
        Y21_val = Y21.value
        Y33_val = Y33.value
        Y31_val = Y31.value
        Y32_val = Y32.value
        
        gamma_val = gamma.value
        mu_val = mu.value
        
        # Calculate controller gains
        # K_ij = Y_ij * Q_j^-1
        K10 = Y11_val @ np.linalg.inv(Q1_val)
        K21 = Y21_val @ np.linalg.inv(Q1_val)
        K20 = Y22_val @ np.linalg.inv(Q2_val) - K21
        K31 = Y31_val @ np.linalg.inv(Q1_val)
        K32 = Y32_val @ np.linalg.inv(Q2_val)
        K30 = Y33_val @ np.linalg.inv(Q3_val) - K31 - K32
        
        # Print results
        print(f"\nOptimization Results:")
        print(f"gamma = {gamma_val}")
        print(f"mu = {mu_val}")
        
        print(f"\nObserver Gains:")
        print(f"L1 =\n{L1}")
        print(f"\nL2 =\n{L2}")
        print(f"\nL3 =\n{L3}")
        
        print(f"\nController Gains:")
        print(f"K10 = {K10}")
        print(f"K20 = {K20}")
        print(f"K21 = {K21}")
        print(f"K30 = {K30}")
        print(f"K31 = {K31}")
        print(f"K32 = {K32}")
        
        # Save results to file
        results = {
            'L1': L1, 'L2': L2, 'L3': L3,
            'K10': K10, 'K20': K20, 'K21': K21,
            'K30': K30, 'K31': K31, 'K32': K32,
            'P1': P1_val, 'P2': P2_val, 'P3': P3_val,
            'Q1': Q1_val, 'Q2': Q2_val, 'Q3': Q3_val,
            'gamma': gamma_val, 'mu': mu_val
        }
        
        np.savez('observer_controller_gains.npz', **results)
        print("\nResults saved to 'observer_controller_gains.npz'")
        
        return results
        
    else:
        print(f'\nLMI is not feasible! (Status: {prob.status})')
        return None

if __name__ == "__main__":
    results = main()
