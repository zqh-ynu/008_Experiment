"""Retrieve county-centre coordinates from the AMap District API.

The script writes the county-centre table used by the population-data
preprocessing workflow.  The API credential is intentionally read from the
local AMAP_API_KEY environment variable so that no secret is stored in source
control.
"""

import os

import pandas as pd
import requests


def get_china_counties(api_key: str) -> pd.DataFrame:
    """Query the AMap District API and return county-level centre coordinates.

    Args:
        api_key: An AMap API credential supplied by the caller.

    Returns:
        A DataFrame containing province, city, county, administrative code, and
        centre-coordinate fields for all county-level records returned by AMap.
    """

    # Request the district hierarchy down to county level using AMap's base
    # district response format.
    url = "https://restapi.amap.com/v3/config/district"
    params = {
        "key": api_key,
        "keywords": "中国",
        "subdistrict": 3,
        "extensions": "base",
    }

    response = requests.get(url, params=params)
    data = response.json()
    county_list = []

    if data["status"] == "1":
        # Traverse province -> city -> county records in the API hierarchy.
        for province in data["districts"][0]["districts"]:
            for city in province["districts"]:
                for county in city["districts"]:
                    county_list.append(
                        {
                            "省份": province["name"],
                            "城市": city["name"],
                            "县区": county["name"],
                            "行政代码": county["adcode"],
                            "中心点坐标": county["center"],
                        }
                    )

    return pd.DataFrame(county_list)


def get_amap_api_key() -> str:
    """Read the local AMap credential without storing it in the repository.

    Returns:
        The non-empty AMAP_API_KEY environment-variable value.

    Raises:
        RuntimeError: If AMAP_API_KEY is not configured in the current process.
    """

    api_key = os.getenv("AMAP_API_KEY")
    if not api_key:
        raise RuntimeError(
            "未设置 AMAP_API_KEY。请在本机环境变量中配置高德 API 密钥后再运行该脚本。"
        )
    return api_key


def main() -> None:
    """Fetch county centres and save them as a UTF-8-with-BOM CSV file."""

    dataframe = get_china_counties(get_amap_api_key())
    dataframe[["经度", "纬度"]] = dataframe["中心点坐标"].str.split(
        ",", expand=True
    )
    dataframe.to_csv("china_counties_coords.csv", index=False, encoding="utf_8_sig")
    print(f"成功获取 {len(dataframe)} 个县级单位坐标！")


if __name__ == "__main__":
    main()
