"""创建 GitHub Release 并上传单文件 EXE 附件。

用法: python _make_release.py <token>
"""
import json
import mimetypes
import os
import sys
import urllib.error
import urllib.parse
import urllib.request

OWNER = "cdh66666"
REPO = "Cable_driven_massage"
TAG = "v1.3.0"
ASSET_NAME = "Y42-PC-Console-v1.3.0.exe"
EXE = r"D:\Cable_driven_massage-main\Y42调试台.exe"
BODY_JSON = r"D:\Cable_driven_massage-main\packaging\_release_body.json"
API = "https://api.github.com"
UPLOADS = "https://uploads.github.com"


def call(url, token, data=None, method="GET", ctype="application/json"):
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", "token " + token)
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("User-Agent", "y42-release-helper")
    if data is not None:
        req.add_header("Content-Type", ctype)
    with urllib.request.urlopen(req, timeout=300) as r:
        raw = r.read()
    return json.loads(raw.decode("utf-8")) if raw else {}


def main():
    token = sys.argv[1].strip()
    with open(BODY_JSON, "r", encoding="utf-8") as f:
        payload = json.load(f)

    base = f"{API}/repos/{OWNER}/{REPO}"
    try:
        rel = call(f"{base}/releases/tags/{TAG}", token)
        print(f"[i] 已存在 Release {TAG} -> {rel['html_url']}")
    except urllib.error.HTTPError as e:
        if e.code != 404:
            raise
        rel = call(
            f"{base}/releases",
            token,
            data=json.dumps(payload).encode("utf-8"),
            method="POST",
        )
        print(f"[+] 已创建 Release {rel['tag_name']} -> {rel['html_url']}")

    # 已存在的同名附件先删掉，保证可重复执行
    for a in rel.get("assets", []):
        if a["name"] == ASSET_NAME:
            call(f"{API}/repos/{OWNER}/{REPO}/releases/assets/{a['id']}", token, method="DELETE")
            print(f"[i] 删除同名旧附件 {ASSET_NAME}")

    size = os.path.getsize(EXE)
    print(f"[i] 上传 {ASSET_NAME} ({size/1048576:.2f} MB) ...")
    with open(EXE, "rb") as f:
        blob = f.read()
    url = (
        f"{UPLOADS}/repos/{OWNER}/{REPO}/releases/{rel['id']}/assets"
        f"?name={urllib.parse.quote(ASSET_NAME)}"
    )
    ctype = mimetypes.guess_type(ASSET_NAME)[0] or "application/octet-stream"
    asset = call(url, token, data=blob, method="POST", ctype=ctype)
    print(f"[+] 上传完成: {asset['name']}  {asset['size']/1048576:.2f} MB")
    print(f"[+] 直链: {asset['browser_download_url']}")
    print(f"[+] 页面: {rel['html_url']}")


if __name__ == "__main__":
    main()
