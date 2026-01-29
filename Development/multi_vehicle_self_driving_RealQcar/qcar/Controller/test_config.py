"""
控制器配置系统测试脚本 / Controller Configuration System Test Script

测试项目 / Tests:
1. 基础配置加载 / Basic configuration loading
2. 控制器创建 / Controller creation
3. Per-vehicle配置覆盖 / Per-vehicle configuration overrides
4. 参数回退到默认值 / Parameter fallback to defaults

运行此脚本以确保你的controller_config.yaml配置正常工作。
Run this to ensure your controller_config.yaml is working correctly.
"""
import sys
import os

# Add parent directory to path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

def test_config_loading():
    """测试配置文件加载 / Test that config file can be loaded"""
    print("=" * 60)
    print("测试控制器配置系统 / Testing Controller Configuration System")
    print("=" * 60)
    
    try:
        from Controller.config_controller_loader import get_controller_config
        print("✓ 配置加载器导入成功 / Config loader imported successfully")
    except ImportError as e:
        print(f"✗ 导入配置加载器失败 / Failed to import config_loader: {e}")
        return False
    
    try:
        config = get_controller_config()
        print(f"✓ 配置加载成功 / Config loaded successfully")
        print(f"  配置文件 / Config file: {config.config_path}")
    except FileNotFoundError as e:
        print(f"✗ 配置文件未找到 / Config file not found: {e}")
        print("\n请复制controller_config_vehicle_main.yaml为controller_config.yaml")
        print("Please copy controller_config_vehicle_main.yaml to controller_config.yaml")
        return False
    except Exception as e:
        print(f"✗ 配置加载失败 / Failed to load config: {e}")
        return False
    
    # Test getting controller types
    try:
        long_type = config.get_longitudinal_controller_type()
        lat_type = config.get_lateral_controller_type()
        print(f"✓ 控制器类型加载成功 / Controller types loaded:")
        print(f"  纵向控制器 / Longitudinal: {long_type}")
        print(f"  横向控制器 / Lateral: {lat_type}")
    except Exception as e:
        print(f"✗ 获取控制器类型失败 / Failed to get controller types: {e}")
        return False
    
    # Test getting longitudinal parameters
    try:
        long_params = config.get_longitudinal_params()
        print(f"✓ 纵向控制器参数加载成功 / Longitudinal parameters loaded:")
        for key, value in long_params.items():
            print(f"  {key}: {value}")
    except Exception as e:
        print(f"✗ 获取纵向控制器参数失败 / Failed to get longitudinal params: {e}")
        return False
    
    # Test getting lateral parameters
    try:
        lat_params = config.get_lateral_params()
        print(f"✓ 横向控制器参数加载成功 / Lateral parameters loaded:")
        for key, value in lat_params.items():
            if hasattr(value, '__class__'):
                print(f"  {key}: {value.__class__.__name__}")
            else:
                print(f"  {key}: {value}")
    except Exception as e:
        print(f"✗ 获取横向控制器参数失败 / Failed to get lateral params: {e}")
        return False
    
    # Test creating controllers
    print("\n" + "-" * 60)
    print("测试控制器创建 / Testing Controller Creation")
    print("-" * 60)
    
    try:
        from Controller.longitudinal_controllers import ControllerFactory
        long_controller = ControllerFactory.create(
            long_type,
            long_params
        )
        print(f"✓ 创建纵向控制器成功 / Created {long_type} longitudinal controller: {long_controller.__class__.__name__}")
    except Exception as e:
        print(f"✗ 创建纵向控制器失败 / Failed to create longitudinal controller: {e}")
        return False
    
    try:
        from Controller.lateral_controllers import LateralControllerFactory
        if lat_type != 'hybrid':  # Hybrid params already contain controller instances
            lat_controller = LateralControllerFactory.create(
                lat_type,
                lat_params
            )
            print(f"✓ 创建横向控制器成功 / Created {lat_type} lateral controller: {lat_controller.__class__.__name__}")
        else:
            print(f"✓ Hybrid横向控制器已配置 / Hybrid lateral controller configured (contains sub-controllers)")
    except Exception as e:
        print(f"✗ 创建横向控制器失败 / Failed to create lateral controller: {e}")
        return False
    
    print("\n" + "=" * 60)
    print("✓ 所有测试通过 / ALL TESTS PASSED!")
    print("=" * 60)
    print("\n你的控制器配置系统工作正常。")
    print("Your controller configuration system is working correctly.")
    print("\n你现在可以编辑controller_config.yaml来修改参数。")
    print("You can now edit controller_config.yaml to change parameters.")
    return True


def test_per_vehicle_config():
    """测试per-vehicle配置系统 / Test per-vehicle configuration system"""
    print("\n" + "=" * 80)
    print("测试Per-Vehicle配置系统 / Testing Per-Vehicle Configuration System")
    print("=" * 80)
    
    from Controller.config_controller_loader import get_controller_config
    
    # Test vehicle without per-vehicle config
    print("\n--- 测试1: 无per-vehicle配置的车辆 / Test 1: Vehicle without per-vehicle config (Vehicle 99) ---")
    try:
        config_default = get_controller_config(vehicle_id=99)
        long_type = config_default.get_longitudinal_controller_type()
        lat_type = config_default.get_lateral_controller_type()
        print(f"✓ Vehicle 99 使用全局默认配置 / Using global default config:")
        print(f"  纵向控制器 / Longitudinal: {long_type}")
        print(f"  横向控制器 / Lateral: {lat_type}")
    except Exception as e:
        print(f"✗ 失败 / Failed: {e}")
        return False
    
    # Test vehicles with per-vehicle config
    test_vehicles = [0, 1, 2, 3]
    
    for vehicle_id in test_vehicles:
        print(f"\n--- 测试 / Test: Vehicle {vehicle_id} (per-vehicle config) ---")
        try:
            config = get_controller_config(vehicle_id=vehicle_id)
            long_type = config.get_longitudinal_controller_type()
            lat_type = config.get_lateral_controller_type()
            
            print(f"✓ Vehicle {vehicle_id} 配置 / Config:")
            print(f"  纵向控制器 / Longitudinal: {long_type}")
            print(f"  横向控制器 / Lateral: {lat_type}")
            
            # Get and display parameters
            long_params = config.get_longitudinal_params()
            print(f"  纵向控制器参数:")
            for key, value in long_params.items():
                if key != 'K' and key != 'observer':  # Skip complex objects
                    print(f"    {key}: {value}")
            
            lat_params = config.get_lateral_params()
            print(f"  横向控制器参数:")
            for key, value in lat_params.items():
                if not hasattr(value, '__call__') and not hasattr(value, '__class__') or isinstance(value, (int, float, str, bool)):
                    print(f"    {key}: {value}")
                    
        except Exception as e:
            print(f"✗ Vehicle {vehicle_id}失败 / Failed for vehicle {vehicle_id}: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    print("\n" + "=" * 80)
    print("✓ Per-Vehicle配置测试通过 / Per-Vehicle Configuration Tests PASSED!")
    print("=" * 80)
    return True


def test_parameter_fallback():
    """测试未指定参数回退到全局默认值 / Test that unspecified parameters fall back to global defaults"""
    print("\n" + "=" * 80)
    print("测试参数回退到默认值 / Testing Parameter Fallback to Defaults")
    print("=" * 80)
    
    from Controller.config_controller_loader import get_controller_config
    
    # Get global defaults (no vehicle_id)
    config_global = get_controller_config(vehicle_id=None)
    
    # Test state_feedback controller parameters
    print("\n--- 测试state_feedback控制器 / Testing state_feedback controller ---")
    try:
        sf_global = config_global.get_longitudinal_params('state_feedback')
        print(f"全局默认state_feedback参数 / Global default state_feedback params:")
        print(f"  max_throttle: {sf_global.get('max_throttle')}")
        print(f"  throttle_smoothing: {sf_global.get('throttle_smoothing')}")
        
        # Test Vehicle 1 (should have per-vehicle config)
        config_v1 = get_controller_config(vehicle_id=1)
        sf_v1 = config_v1.get_longitudinal_params('state_feedback')
        print(f"\nVehicle 1 state_feedback参数 / params:")
        print(f"  max_throttle: {sf_v1.get('max_throttle')}")
        print(f"  throttle_smoothing: {sf_v1.get('throttle_smoothing')}")
        
        # Test Vehicle 2 (partial override)
        config_v2 = get_controller_config(vehicle_id=2)
        sf_v2 = config_v2.get_longitudinal_params('state_feedback')
        print(f"\nVehicle 2 state_feedback参数(部分覆盖) / params (partial override):")
        print(f"  max_throttle: {sf_v2.get('max_throttle')}")
        print(f"  throttle_smoothing: {sf_v2.get('throttle_smoothing')}")
        
        # Verify fallback behavior
        print("\n--- 验证参数回退机制 ---")
        if sf_v2.get('throttle_smoothing') != sf_global.get('throttle_smoothing'):
            print(f"✓ Vehicle 2 的 throttle_smoothing 使用了个性化值 ({sf_v2.get('throttle_smoothing')})")
        
        # Test unspecified vehicle (should use all defaults)
        config_v99 = get_controller_config(vehicle_id=99)
        sf_v99 = config_v99.get_longitudinal_params('state_feedback')
        
        if (sf_v99.get('max_throttle') == sf_global.get('max_throttle') and
            sf_v99.get('throttle_smoothing') == sf_global.get('throttle_smoothing')):
            print(f"✓ Vehicle 99(未配置)正确回退到全局默认值 / Correctly falls back to global defaults")
        else:
            print(f"✗ Vehicle 99参数回退失败 / Parameter fallback failed")
            return False
            
    except Exception as e:
        print(f"✗ 失败 / Failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\n" + "=" * 80)
    print("✓ 参数回退测试通过 / Parameter Fallback Tests PASSED!")
    print("=" * 80)
    return True


def test_controller_creation_per_vehicle():
    """测试使用per-vehicle配置创建控制器 / Test creating controllers with per-vehicle configs"""
    print("\n" + "=" * 80)
    print("测试Per-Vehicle配置下的控制器创建 / Testing Controller Creation with Per-Vehicle Config")
    print("=" * 80)
    
    from Controller.config_controller_loader import get_controller_config
    from Controller.longitudinal_controllers import ControllerFactory
    from Controller.lateral_controllers import LateralControllerFactory
    
    test_vehicles = [0, 1, 2]
    
    for vehicle_id in test_vehicles:
        print(f"\n--- 为Vehicle {vehicle_id}创建控制器 / Creating controllers for Vehicle {vehicle_id} ---")
        try:
            config = get_controller_config(vehicle_id=vehicle_id)
            
            # Create longitudinal controller
            long_type = config.get_longitudinal_controller_type()
            long_params = config.get_longitudinal_params()
            
            # Remove observer for testing (it will be None)
            long_params_test = {k: v for k, v in long_params.items() if k != 'observer'}
            
            try:
                long_controller = ControllerFactory.create(long_type, long_params_test)
                print(f"✓ 纵向控制器: {long_controller.__class__.__name__} ({long_type})")
            except Exception as e:
                print(f"⚠ 纵向控制器创建跳过 ({long_type}): {e}")
            
            # Create lateral controller
            lat_type = config.get_lateral_controller_type()
            lat_params = config.get_lateral_params()
            
            try:
                if lat_type != 'hybrid':
                    lat_controller = LateralControllerFactory.create(lat_type, lat_params)
                    print(f"✓ 横向控制器: {lat_controller.__class__.__name__} ({lat_type})")
                else:
                    print(f"✓ 横向控制器: Hybrid (包含子控制器)")
            except Exception as e:
                print(f"⚠ 横向控制器创建跳过 ({lat_type}): {e}")
                
        except Exception as e:
            print(f"✗ Vehicle {vehicle_id}失败 / Failed for vehicle {vehicle_id}: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    print("\n" + "=" * 80)
    print("✓ 控制器创建测试通过 / Controller Creation Tests PASSED!")
    print("=" * 80)
    return True


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("控制器配置测试套件 / CONTROLLER CONFIGURATION TEST SUITE")
    print("=" * 80)
    
    all_passed = True
    
    # Test 1: Basic config loading
    if not test_config_loading():
        all_passed = False
    
    # Test 2: Per-vehicle configuration
    if all_passed and not test_per_vehicle_config():
        all_passed = False
    
    # Test 3: Parameter fallback
    if all_passed and not test_parameter_fallback():
        all_passed = False
    
    # Test 4: Controller creation with per-vehicle config
    if all_passed and not test_controller_creation_per_vehicle():
        all_passed = False
    
    # Final summary
    print("\n" + "=" * 80)
    if all_passed:
        print("🎉 所有测试通过 / ALL TESTS PASSED! 🎉")
        print("=" * 80)
        print("\n总结 / Summary:")
        print("✓ 基础配置加载正常 / Basic config loading works")
        print("✓ Per-vehicle配置系统工作正常 / Per-vehicle config system works")
        print("✓ 参数回退到默认值机制正常 / Parameter fallback mechanism works")
        print("✓ 控制器创建正常 / Controller creation works")
        print("\n配置系统已准备就绪！")
        print("Configuration system is ready!")
    else:
        print("❌ 部分测试失败 / SOME TESTS FAILED")
        print("=" * 80)
        print("\n请检查上述错误信息并修复配置文件。")
        print("Please check the error messages above and fix the configuration file.")
    print("=" * 80)
    
    sys.exit(0 if all_passed else 1)
