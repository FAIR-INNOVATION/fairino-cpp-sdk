/* mbedtls_config.h - TCP/UDP (DTLS) 客户端 */
#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

// ---- 基础平台 ----
#define MBEDTLS_HAVE_TIME
#define MBEDTLS_HAVE_TIME_DATE

// ---- 随机数 (必需) ----
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_CTR_DRBG_C

// ---- 哈希与对称加密 ----
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA512_C          // ECDHE_RSA套件可能需要
#define MBEDTLS_AES_C
#define MBEDTLS_GCM_C             // 推荐使用GCM模式
#define MBEDTLS_CIPHER_C
#define MBEDTLS_MD_C

// ---- 非对称加密与公钥基础设施 ----
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_OID_C
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21         // 支持RSA-PSS
#define MBEDTLS_RSA_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C        // 解析私钥/公钥
#define MBEDTLS_PK_WRITE_C        // 如需写出密钥

// ---- 椭圆曲线 (支持ECDHE前向保密) ----
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED

// ---- X.509证书 (解析与验证) ----
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_PEM_WRITE_C
#define MBEDTLS_BASE64_C          // PEM解析的底层依赖

// ---- TLS/DTLS 核心 ----
#define MBEDTLS_SSL_TLS_C         // 修正：TLS核心
#define MBEDTLS_SSL_CLI_C         // 修正：TLS客户端
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_PROTO_DTLS    // 为UDP启用DTLS
#define MBEDTLS_TIMING_C

// ---- 密钥交换模式 ----
#define MBEDTLS_KEY_EXCHANGE_RSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED

// ---- 指定密码套件 (示例) ----
#define MBEDTLS_SSL_CIPHERSUITES \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256

// ---- 网络抽象层 ----
#define MBEDTLS_NET_C

// ---- 错误字符串 (调试用) ----
#define MBEDTLS_ERROR_C

// ---- PSA Crypto (推荐) ----
#define MBEDTLS_PSA_CRYPTO_C
// #define MBEDTLS_USE_PSA_CRYPTO  // 可选，让TLS内部使用PSA

#define MBEDTLS_FS_IO

#include "mbedtls/check_config.h"
#endif /* MBEDTLS_CONFIG_H */