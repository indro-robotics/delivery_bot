import requests

response = requests.get('https://api.weatherunlocked.com/api/resortforecast/54887593?app_id=d5a58e66&app_key=5b2d31f55bdbdbc1cb8de17b320858dd')

print(response.json())