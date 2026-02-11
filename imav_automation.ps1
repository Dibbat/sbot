# IMAV 2017 Virtual Challenge - Complete Automation Script
# This PowerShell script automates the entire setup and launch process

param(
    [string]$Action = "start"  # "start", "stop", "restart", "logs", "status"
)

$ContainerName = "imav_2017_dev"
$ProjectPath = "d:\IMAV_2017_Virtual_Challenge"
$BridgePort = 9999

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "IMAV 2017 Virtual Challenge - Automation Script" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

function Check-Docker {
    try {
        $status = docker ps | Select-String $ContainerName
        return $null -ne $status
    }
    catch {
        Write-Host "[ERROR] Docker not running or container not found" -ForegroundColor Red
        return $false
    }
}

function Start-System {
    Write-Host "[1/5] Checking Docker container..." -ForegroundColor Yellow
    
    if (-not (Check-Docker)) {
        Write-Host "[INFO] Container not running. Starting docker-compose..." -ForegroundColor Cyan
        cd $ProjectPath
        docker-compose up -d
        Start-Sleep -Seconds 3
    }
    
    if (-not (Check-Docker)) {
        Write-Host "[ERROR] Failed to start container" -ForegroundColor Red
        exit 1
    }
    Write-Host "[✓] Docker container is running" -ForegroundColor Green
    
    Write-Host ""
    Write-Host "[2/5] Rebuilding ROS2 workspace..." -ForegroundColor Yellow
    docker exec $ContainerName bash -c "cd /app && source /opt/ros/humble/setup.bash && colcon build --packages-select imav_2017 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5"
    Write-Host "[✓] Workspace rebuilt" -ForegroundColor Green
    
    Write-Host ""
    Write-Host "[3/5] Starting Gazebo simulator..." -ForegroundColor Yellow
    docker exec -d $ContainerName bash -c "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && export ROS_DOMAIN_ID=25 && ros2 launch imav_2017 imav_indoor.launch.py" | Out-Null
    Start-Sleep -Seconds 5
    Write-Host "[✓] Gazebo launching in background" -ForegroundColor Green
    
    Write-Host ""
    Write-Host "[4/5] Spawning drone in Gazebo..." -ForegroundColor Yellow
    docker exec -d $ContainerName bash -c "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && export ROS_DOMAIN_ID=25 && sleep 2 && ros2 launch imav_2017 spawn_quadrotor.launch.py" | Out-Null
    Start-Sleep -Seconds 4
    Write-Host "[✓] Drone spawning in background" -ForegroundColor Green
    
    Write-Host ""
    Write-Host "[5/5] Starting TCP Bridge Server..." -ForegroundColor Yellow
    docker exec -d $ContainerName bash -c "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && python3 /app/ros2_tcp_bridge.py" | Out-Null
    Start-Sleep -Seconds 2
    Write-Host "[✓] TCP Bridge started on port $BridgePort" -ForegroundColor Green
    
    Write-Host ""
    Write-Host "================================================" -ForegroundColor Green
    Write-Host "✓ SYSTEM READY FOR MATLAB CONNECTION" -ForegroundColor Green
    Write-Host "================================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Connection Details:" -ForegroundColor Cyan
    Write-Host "  Host: 192.168.0.38" -ForegroundColor White
    Write-Host "  Port: $BridgePort" -ForegroundColor White
    Write-Host ""
    Write-Host "MATLAB Example:" -ForegroundColor Cyan
    Write-Host "  client = ROS2BridgeClient('192.168.0.38', $BridgePort);" -ForegroundColor White
    Write-Host "  client.connect();" -ForegroundColor White
    Write-Host "  odom = client.receive_message(2);" -ForegroundColor White
    Write-Host "  client.publish_velocity(0.5, 0, 0, 0, 0, 0);" -ForegroundColor White
    Write-Host ""
}

function Stop-System {
    Write-Host "[INFO] Stopping Docker container..." -ForegroundColor Yellow
    cd $ProjectPath
    docker-compose down
    Write-Host "[✓] System stopped" -ForegroundColor Green
}

function Restart-System {
    Write-Host "[INFO] Restarting system..." -ForegroundColor Yellow
    Stop-System
    Start-Sleep -Seconds 2
    Start-System
}

function Show-Logs {
    Write-Host "[INFO] Showing container logs..." -ForegroundColor Yellow
    docker logs $ContainerName --tail 100 -f
}

function Show-Status {
    Write-Host "[INFO] System Status:" -ForegroundColor Yellow
    Write-Host ""
    
    # Container status
    Write-Host "Container Status:" -ForegroundColor Cyan
    docker ps | Select-String $ContainerName | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
    
    Write-Host ""
    Write-Host "Port Status:" -ForegroundColor Cyan
    $portTest = Test-NetConnection -ComputerName 192.168.0.38 -Port $BridgePort -WarningAction SilentlyContinue
    if ($portTest.TcpTestSucceeded) {
        Write-Host "  [✓] Port $BridgePort is accessible" -ForegroundColor Green
    } else {
        Write-Host "  [✗] Port $BridgePort not responding" -ForegroundColor Red
    }
    
    Write-Host ""
    Write-Host "ROS2 Topics (sample):" -ForegroundColor Cyan
    docker exec $ContainerName bash -c "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && ros2 topic list 2>/dev/null | head -5" 2>/dev/null | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
}

# Main execution
Write-Host ""
switch ($Action.ToLower()) {
    "start" { Start-System }
    "stop" { Stop-System }
    "restart" { Restart-System }
    "logs" { Show-Logs }
    "status" { Show-Status }
    default {
        Write-Host "Usage: .\imav_automation.ps1 [start|stop|restart|logs|status]" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Examples:" -ForegroundColor Cyan
        Write-Host "  .\imav_automation.ps1 start        # Start complete system" -ForegroundColor White
        Write-Host "  .\imav_automation.ps1 stop         # Stop everything" -ForegroundColor White
        Write-Host "  .\imav_automation.ps1 restart      # Restart system" -ForegroundColor White
        Write-Host "  .\imav_automation.ps1 status       # Check system status" -ForegroundColor White
        Write-Host "  .\imav_automation.ps1 logs         # View live logs" -ForegroundColor White
    }
}
