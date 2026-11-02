FROM python@sha256:1aaa65a85fda306ffb8b910824d4e93bdce61e212c7e87168123ea3073b41a1a
WORKDIR /app
COPY solution/solution.py solution/weather_groups.json solution/calibration.json /app/
USER 65532:65532
ENTRYPOINT ["python", "-u", "/app/solution.py"]
