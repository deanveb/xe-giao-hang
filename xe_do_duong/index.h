const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Xe dò đường - Hệ thống định tuyến thông minh</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    :root {
      --primary: #00f7ff;
      --secondary: #ff00aa;
      --bg-dark: #0a0a1a;
      --bg-light: #1a1a2e;
      --text: #e0e0ff;
      --text-dim: #a0a0c0;
    }
    
    body {
      font-family: 'Segoe UI', 'Roboto', sans-serif;
      background-color: var(--bg-dark);
      color: var(--text);
      margin: 0;
      padding: 20px;
      background-image: 
        radial-gradient(circle at 25% 25%, rgba(0, 247, 255, 0.05) 0%, transparent 50%),
        radial-gradient(circle at 75% 75%, rgba(255, 0, 170, 0.05) 0%, transparent 50%);
      min-height: 100vh;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    
    .container {
      max-width: 800px;
      padding: 20px;
      background-color: var(--bg-light);
      border-radius: 12px;
      box-shadow: 0 0 30px rgba(0, 247, 255, 0.1),
                  0 0 15px rgba(255, 0, 170, 0.1);
      border: 1px solid rgba(0, 247, 255, 0.2);
    }
    
    .header {
      display: flex;
      align-items: center;
      margin-bottom: 30px;
      padding-bottom: 15px;
      border-bottom: 1px solid rgba(0, 247, 255, 0.3);
    }
    
    .logo {
      width: 50px;
      height: 50px;
      margin-right: 15px;
      background: linear-gradient(135deg, var(--primary), var(--secondary));
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: bold;
      font-size: 20px;
      color: var(--bg-dark);
    }
    
    h1 {
      margin: 0;
      font-weight: 300;
      font-size: 24px;
      letter-spacing: 1px;
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      -webkit-background-clip: text;
      background-clip: text;
      color: transparent;
    }
    
    .route-form {
      display: flex;
      flex-wrap: wrap;
      gap: 15px;
      align-items: center;
      margin-bottom: 30px;
      justify-content: center;
    }
    
    .route-form > div {
      flex: 1;
      min-width: 180px;
    }
    
    .route-form .arrow {
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--primary);
      font-size: 24px;
      margin: 0 10px;
      align-self: flex-end;
    }
    
    label {
      color: var(--text-dim);
      font-size: 14px;
      text-transform: uppercase;
      letter-spacing: 1px;
      display: block;
      margin-bottom: 5px;
    }
    
    select, input[type="submit"], button {
      background-color: rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(0, 247, 255, 0.3);
      color: var(--text);
      padding: 12px 15px;
      border-radius: 6px;
      font-size: 16px;
      transition: all 0.3s ease;
      width: 100%;
    }
    
    select {
      appearance: none;
      background-image: url("data:image/svg+xml;charset=UTF-8,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='%2300f7ff'%3e%3cpath d='M7 10l5 5 5-5z'/%3e%3c/svg%3e");
      background-repeat: no-repeat;
      background-position: right 10px center;
      background-size: 15px;
      padding-right: 35px;
    }
    
    select:focus {
      outline: none;
      border-color: var(--primary);
      box-shadow: 0 0 0 2px rgba(0, 247, 255, 0.2);
    }
    
    input[type="submit"], button {
      background: linear-gradient(135deg, var(--primary), var(--secondary));
      color: var(--bg-dark);
      font-weight: bold;
      text-transform: uppercase;
      letter-spacing: 1px;
      cursor: pointer;
      border: none;
    }
    
    input[type="submit"]:hover, button:hover {
      transform: translateY(-2px);
      box-shadow: 0 5px 15px rgba(0, 247, 255, 0.3);
    }
    
    .progress-container {
      margin: 30px 0;
      padding: 20px;
      background-color: rgba(0, 0, 0, 0.2);
      border-radius: 8px;
      border: 1px solid rgba(0, 247, 255, 0.2);
    }
    
    progress {
      width: 100%;
      height: 10px;
      margin-top: 10px;
      appearance: none;
      border: none;
      background-color: rgba(0, 247, 255, 0.1);
      border-radius: 5px;
    }
    
    progress::-webkit-progress-bar {
      background-color: rgba(0, 247, 255, 0.1);
      border-radius: 5px;
    }
    
    progress::-webkit-progress-value {
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      border-radius: 5px;
      box-shadow: 0 0 10px rgba(0, 247, 255, 0.5);
    }
    
    .map-container {
      height: 300px;
      background-color: rgba(0, 0, 0, 0.3);
      border-radius: 8px;
      border: 1px solid rgba(0, 247, 255, 0.3);
      position: relative;
      overflow: hidden;
      margin-top: 20px;
    }
    
    .map-placeholder {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      color: var(--text-dim);
      text-align: center;
    }
    
    .grid-lines {
      position: absolute;
      width: 100%;
      height: 100%;
      background-image: 
        linear-gradient(rgba(0, 247, 255, 0.1) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0, 247, 255, 0.1) 1px, transparent 1px);
      background-size: 30px 30px;
    }
    
    .remove-btn {
      background: linear-gradient(135deg, #ff4d4d, #ff1a1a);
      margin-top: 10px;
    }
    
    @media (max-width: 768px) {
      .route-form {
        flex-direction: column;
      }
      
      .route-form .arrow {
        margin: 0px 0;
        align-self: center;
      }
      
      .route-form > div {
        width: 100%;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div class="logo">🚙</div>
      <h1>HỆ THỐNG ĐỊNH TUYẾN XE DÒ ĐƯỜNG</h1>
    </div>
    
    <form method="get" class="route-form">
      <div>
        <label for="from">Điểm xuất phát</label>
        <select name="from" id="from" class="text-center">
          <option value='{"pos0":"6","pos1":"1"}'>Xưởng A</option>
          <option value='{"pos0":"6","pos1":"6"}'>Xưởng B</option>
          <option value='{"pos0":"1","pos1":"6"}'>Xưởng C</option>
        </select>
      </div>

      <div class="arrow">→</div>

      <div>
        <label for="to">Điểm đến</label>
        <select name="to" id="to" class="text-center">
          <option value='{"pos0":"6","pos1":"1"}'>Xưởng A</option>
          <option value='{"pos0":"6","pos1":"6"}' selected>Xưởng B</option>
          <option value='{"pos0":"1","pos1":"6"}'>Xưởng C</option>
        </select>
      </div>
      
      <input type="submit" value="BẮT ĐẦU">
    </form>
    
    <div id="additional-destinations" class="flex items-center justify-center w-fit h-fit border rounded-md space-y-4 mx-auto p-1">
      <button type="button" onclick="addDestination()" class="mt-0">Thêm điểm đến</button>
    </div>
    
    <div class="progress-container">
      <label>Tiến trình vận chuyển</label>
      <progress value="70" max="100"></progress>
      <div style="display: flex; justify-content: space-between; margin-top: 5px;">
        <span style="font-size: 12px; color: var(--text-dim);font-weight: bold;">0%</span>
        <span style="font-size: 12px; color: var(--text-dim);font-weight: bold;">25%</span>
        <span style="font-size: 12px; color: var(--text-dim); font-weight: bold;">50%</span>
        <span style="font-size: 12px; color: var(--text-dim); font-weight: bold;">75%</span>
        <span style="font-size: 12px; color: var(--text-dim);font-weight: bold;">100%</span>
      </div>
    </div>
    
    <!-- <div class="map-container">
      <div class="grid-lines"></div>
      <div class="map-placeholder">
        <svg width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="color: var(--primary); margin-bottom: 10px;">
          <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"></path>
          <circle cx="12" cy="10" r="3"></circle>
        </svg>
        <p>Bản đồ hiển thị vị trí xe sẽ xuất hiện tại đây</p>
      </div>
    </div>
  </div> -->

  <script>
    function addDestination() {
      const container = document.getElementById('additional-destinations');
      const newSelect = document.createElement('div');
      newSelect.innerHTML = `
        <label for="to-extra" class="block">Điểm đến tiếp theo</label>
        <select name="to-extra" class="w-full">
          <option value="{"pos0":"6","pos1":"1"}">Xưởng A</option>
          <option value="{"pos0":"6","pos1":"6"}">Xưởng B</option>
          <option value="{"pos0":"1","pos1":"6"}">Xưởng C</option>
        </select>
        <button type="button" onclick="this.parentElement.remove()" class="remove-btn">Xóa</button>
      `;
      container.appendChild(newSelect);
    }
  </script>
</body>
</html>
)rawliteral";
