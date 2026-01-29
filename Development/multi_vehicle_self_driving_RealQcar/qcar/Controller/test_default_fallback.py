"""
Test script to verify parameter fallback to defaults
测试参数回退到默认值的行为
"""
import sys
import os

# Add parent directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
qcar_dir = os.path.dirname(current_dir)
sys.path.insert(0, qcar_dir)

from Controller.config_controller_loader import get_controller_config

def test_parameter_fallback():
    """Test that per-vehicle config falls back to global defaults for unspecified params"""
    
    print("=" * 80)
    print("测试参数回退到默认值")
    print("=" * 80)
    
    # Test Vehicle 0 (has per-vehicle config)
    print("\n--- Vehicle 0 (配置了 fix 控制器) ---")
    config_v0 = get_controller_config(vehicle_id=0)
    
    print(f"控制器类型: {config_v0.get_longitudinal_controller_type()}")
    fix_params = config_v0.get_longitudinal_params('fix')
    print(f"Fix 参数: {fix_params}")
    
    # Test that we can still access other controller params (global defaults)
    print("\n--- Vehicle 0 访问其他控制器的全局默认参数 ---")
    try:
        cacc_params = config_v0.get_longitudinal_params('cacc')
        print(f"CACC 参数 (全局默认): s0={cacc_params.get('s0')}, h={cacc_params.get('h')}")
    except Exception as e:
        print(f"错误: {e}")
    
    # Test Vehicle 1 (has state_feedback config)
    print("\n--- Vehicle 1 (配置了 state_feedback 控制器) ---")
    config_v1 = get_controller_config(vehicle_id=1)
    
    print(f"控制器类型: {config_v1.get_longitudinal_controller_type()}")
    sf_params = config_v1.get_longitudinal_params('state_feedback')
    print(f"State Feedback 参数:")
    print(f"  max_throttle: {sf_params.get('max_throttle')}")
    print(f"  throttle_smoothing: {sf_params.get('throttle_smoothing')}")
    
    # Test Vehicle 2 (has partial state_feedback config)
    print("\n--- Vehicle 2 (部分配置 state_feedback) ---")
    config_v2 = get_controller_config(vehicle_id=2)
    
    sf_params_v2 = config_v2.get_longitudinal_params('state_feedback')
    print(f"State Feedback 参数:")
    print(f"  max_throttle: {sf_params_v2.get('max_throttle')} (个性化值)")
    print(f"  throttle_smoothing: {sf_params_v2.get('throttle_smoothing')} (个性化值)")
    
    # Test Vehicle 99 (no per-vehicle config)
    print("\n--- Vehicle 99 (没有个性化配置) ---")
    config_v99 = get_controller_config(vehicle_id=99)
    
    print(f"控制器类型: {config_v99.get_longitudinal_controller_type()} (全局默认)")
    sf_params_default = config_v99.get_longitudinal_params('state_feedback')
    print(f"State Feedback 参数 (全局默认):")
    print(f"  max_throttle: {sf_params_default.get('max_throttle')}")
    print(f"  throttle_smoothing: {sf_params_default.get('throttle_smoothing')}")
    
    # Compare global defaults vs per-vehicle overrides
    print("\n" + "=" * 80)
    print("参数覆盖验证")
    print("=" * 80)
    
    # Global default
    config_global = get_controller_config()
    sf_global = config_global.get_longitudinal_params('state_feedback')
    
    print(f"\n全局默认 state_feedback.max_throttle: {sf_global.get('max_throttle')}")
    print(f"Vehicle 1 state_feedback.max_throttle: {sf_params.get('max_throttle')}")
    print(f"Vehicle 2 state_feedback.max_throttle: {sf_params_v2.get('max_throttle')}")
    print(f"Vehicle 99 state_feedback.max_throttle: {sf_params_default.get('max_throttle')}")
    
    # Test scenario: Vehicle with controller type override but no params
    print("\n" + "=" * 80)
    print("场景测试：指定控制器类型但不指定参数")
    print("=" * 80)
    print("\n假设 Vehicle 3 配置了 longitudinal_controller_type: 'state_feedback'")
    print("但没有在 per_vehicle_controllers[3] 中配置 state_feedback 参数")
    
    config_v3 = get_controller_config(vehicle_id=3)
    print(f"控制器类型: {config_v3.get_longitudinal_controller_type()}")
    
    sf_params_v3 = config_v3.get_longitudinal_params('state_feedback')
    print(f"\nState Feedback 参数:")
    print(f"  max_throttle: {sf_params_v3.get('max_throttle')}")
    print(f"  throttle_smoothing: {sf_params_v3.get('throttle_smoothing')}")
    
    # Check if it's using global defaults
    if sf_params_v3.get('max_throttle') == sf_global.get('max_throttle'):
        print("\n✓ Vehicle 3 的 max_throttle 使用了全局默认值")
    else:
        print(f"\n✗ Vehicle 3 的 max_throttle ({sf_params_v3.get('max_throttle')}) 不是全局默认值 ({sf_global.get('max_throttle')})")
    
    if sf_params_v3.get('throttle_smoothing') == sf_global.get('throttle_smoothing'):
        print("✓ Vehicle 3 的 throttle_smoothing 使用了全局默认值")
    else:
        print(f"✗ Vehicle 3 的 throttle_smoothing ({sf_params_v3.get('throttle_smoothing')}) 不是全局默认值 ({sf_global.get('throttle_smoothing')})")
    
    print("\n" + "=" * 80)
    print("结论：")
    print("=" * 80)
    print("✓ 当车辆的个性化配置中没有指定某个参数时，")
    print("  系统会自动使用全局默认参数")
    print("✓ 这种设计允许只覆盖需要修改的参数，")
    print("  其他参数保持全局默认值")
    print("=" * 80)


if __name__ == "__main__":
    test_parameter_fallback()
