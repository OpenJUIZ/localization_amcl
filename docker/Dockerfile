# syntax=docker/dockerfile:1
# localization_amcl のイメージ。リポジトリのルートを context にしてビルドする(サブモジュールも取ってくること):
#   git submodule update --init
#   docker build -f docker/Dockerfile -t localization_amcl .
#
# C++ のプラグインは C ABI なので、juiz は pip の juiz-cli をそのまま使う。
# mobile_sim のイメージは juiz run ではなく juiz -f で動くので、こちらも conf の HTTP の subsystem でつなぐ。
# juiz_sdk の版は juiz と揃える(CMakeLists.txt の LOCALIZATION_AMCL_JUIZ_SDK_VERSION と app/juizapp.yaml の juiz)。
ARG JUIZ_VERSION=0.2.7

# プラグインをビルドしてテストする
FROM debian:bookworm AS build
ARG JUIZ_VERSION
RUN apt-get update \
    && apt-get install -y --no-install-recommends cmake gcc g++ make curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN sdk=$(scripts/fetch_juiz_sdk.sh "${JUIZ_VERSION}") \
    && target=$(basename "$sdk" | sed -E 's/^juiz_sdk-[^-]+-//') \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$sdk" \
         -DLOCALIZATION_AMCL_JUIZ_SDK_VERSION="${JUIZ_VERSION}" \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure \
    && mkdir -p "/plugins/$target" \
    && cp build/src/component/liblocalization_amcl.so build/src/component/libjuiz_sdk.so "/plugins/$target/"

FROM python:3.12-slim-bookworm
ARG JUIZ_VERSION
RUN pip install --no-cache-dir "juiz-cli==${JUIZ_VERSION}"
# app/ と同じ形にする(conf からの相対パスでプラグインとブラウザ画面を探す)
COPY app/app.conf /opt/localization_amcl/
COPY app/web /opt/localization_amcl/web
COPY --from=build /plugins /opt/localization_amcl/plugins
COPY LICENSE-MIT LICENSE-APACHE /opt/localization_amcl/
COPY third_party/amcl/LICENSE /opt/localization_amcl/LICENSE-amcl
COPY app/assets /opt/localization_amcl/assets
COPY docker/entrypoint.sh /usr/local/bin/localization_amcl-entrypoint
# conf の地図 (assets/maps/default.yaml) は JUIZ_APP_DIR から探す。ほかの地図は -v ./maps:/maps で渡し、
# 環境変数 AMCL_MAP=/maps/<名前>.yaml か、画面の「ファイルを読む」で /maps/<名前>.yaml を読む
ENV JUIZ_APP_DIR=/opt/localization_amcl
WORKDIR /maps
VOLUME ["/maps"]
# つなぐ mobile_sim(juiz の HTTP)。mobile_sim からは、こちらが接続してきたアドレスの 8002 番にトピックが届く
ENV MOBILE_SIM_URL=http://mobile_sim:8000
EXPOSE 8002
ENTRYPOINT ["localization_amcl-entrypoint"]
