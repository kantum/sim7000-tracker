/******************************************************************************
 * Copyright 2018 Google
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
// This file contains your configuration used to connect to Cloud IoT Core

#include "module_config.h"

const char *apn = APN;
const char *gprsUser = "";
const char *gprsPass = "";

// Cloud iot details.
const char *project_id = GCLOUD_PROJECT_ID;
const char *location = GCLOUD_LOCATION;
const char *registry_id = GCLOUD_REGISTRY_ID;
const char *device_id = GCLOUD_DEVICE_ID;

// Configuration for NTP
const char* ntp_primary = "pool.ntp.org";
const char* ntp_secondary = "time.nist.gov";

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// To get the private key run (where private-key.pem is the ec private key
// used to create the certificate uploaded to google cloud iot):
// openssl ec -in <private-key.pem> -noout -text
// and copy priv: part.
// The key length should be exactly the same as the key length bellow (32 pairs
// of hex digits). If it's bigger and it starts with "00:" delete the "00:". If
// it's smaller add "00:" to the start. If it's too big or too small something
// is probably wrong with your key.
const char *private_key_str =
"7c:f0:09:91:44:48:99:3f:f0:60:fa:bc:42:af:bc:"
"ea:fd:a4:97:eb:95:72:40:f6:1f:0c:e3:17:0c:33:"
"a1:4c";

// Time (seconds) to expire token += 20 minutes for drift
const int jwt_exp_secs = 60*20; // Maximum 24H (3600*24)

// To get the certificate for your region run:
//   openssl s_client -showcerts -connect mqtt.googleapis.com:8883
// for standard mqtt or for LTS:
//   openssl s_client -showcerts -connect mqtt.2030.ltsapis.goog:8883
// Copy the certificate (all lines between and including ---BEGIN CERTIFICATE---
// and --END CERTIFICATE--) to root.cert and put here on the root_cert variable.

// https://cloud.google.com/iot/docs/how-tos/mqtt-bridge#downloading_mqtt_server_certificates
// gtsltsr.pem 

const char *root_cert =
"-----BEGIN CERTIFICATE-----\n"
"MIIDujCCAqKgAwIBAgILBAAAAAABD4Ym5g0wDQYJKoZIhvcNAQEFBQAwTDEgMB4G\n"
"A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjIxEzARBgNVBAoTCkdsb2JhbFNp\n"
"Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDYxMjE1MDgwMDAwWhcNMjExMjE1\n"
"MDgwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMjETMBEG\n"
"A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI\n"
"hvcNAQEBBQADggEPADCCAQoCggEBAKbPJA6+Lm8omUVCxKs+IVSbC9N/hHD6ErPL\n"
"v4dfxn+G07IwXNb9rfF73OX4YJYJkhD10FPe+3t+c4isUoh7SqbKSaZeqKeMWhG8\n"
"eoLrvozps6yWJQeXSpkqBy+0Hne/ig+1AnwblrjFuTosvNYSuetZfeLQBoZfXklq\n"
"tTleiDTsvHgMCJiEbKjNS7SgfQx5TfC4LcshytVsW33hoCmEofnTlEnLJGKRILzd\n"
"C9XZzPnqJworc5HGnRusyMvo4KD0L5CLTfuwNhv2GXqF4G3yYROIXJ/gkwpRl4pa\n"
"zq+r1feqCapgvdzZX99yqWATXgAByUr6P6TqBwMhAo6CygPCm48CAwEAAaOBnDCB\n"
"mTAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUm+IH\n"
"V2ccHsBqBt5ZtJot39wZhi4wNgYDVR0fBC8wLTAroCmgJ4YlaHR0cDovL2NybC5n\n"
"bG9iYWxzaWduLm5ldC9yb290LXIyLmNybDAfBgNVHSMEGDAWgBSb4gdXZxwewGoG\n"
"3lm0mi3f3BmGLjANBgkqhkiG9w0BAQUFAAOCAQEAmYFThxxol4aR7OBKuEQLq4Gs\n"
"J0/WwbgcQ3izDJr86iw8bmEbTUsp9Z8FHSbBuOmDAGJFtqkIk7mpM0sYmsL4h4hO\n"
"291xNBrBVNpGP+DTKqttVCL1OmLNIG+6KYnX3ZHu01yiPqFbQfXf5WRDLenVOavS\n"
"ot+3i9DAgBkcRcAtjOj4LaR0VknFBbVPFd5uRHg5h6h+u/N5GJG79G+dwfCMNYxd\n"
"AfvDbbnvRG15RjF+Cv6pgsH/76tuIMRQyV+dTZsXjAzlAcmgQWpzU/qlULRuJQ/7\n"
"TBj0/VLZjmmx6BEP3ojY+x1J96relc8geMJgEtslQIxq/H5COEBkEveegeGTLg==\n"
"-----END CERTIFICATE-----";

//const char *root_cert_primary =
//"-----BEGIN CERTIFICATE-----\n"
//"MIIBxTCCAWugAwIBAgINAfD3nVndblD3QnNxUDAKBggqhkjOPQQDAjBEMQswCQYD\n"
//"VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzERMA8G\n"
//"A1UEAxMIR1RTIExUU1IwHhcNMTgxMTAxMDAwMDQyWhcNNDIxMTAxMDAwMDQyWjBE\n"
//"MQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExM\n"
//"QzERMA8GA1UEAxMIR1RTIExUU1IwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAATN\n"
//"8YyO2u+yCQoZdwAkUNv5c3dokfULfrA6QJgFV2XMuENtQZIG5HUOS6jFn8f0ySlV\n"
//"eORCxqFyjDJyRn86d+Iko0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUw\n"
//"AwEB/zAdBgNVHQ4EFgQUPv7/zFLrvzQ+PfNA0OQlsV+4u1IwCgYIKoZIzj0EAwID\n"
//"SAAwRQIhAPKuf/VtBHqGw3TUwUIq7TfaExp3bH7bjCBmVXJupT9FAiBr0SmCtsuk\n"
//"miGgpajjf/gFigGM34F9021bCWs1MbL0SA==\n"
//"-----END CERTIFICATE-----";
//
//const char *root_cert_backup =
//"-----BEGIN CERTIFICATE-----\n"
//"MIIB3DCCAYOgAwIBAgINAgPlfvU/k/2lCSGypjAKBggqhkjOPQQDAjBQMSQwIgYD\n"
//"VQQLExtHbG9iYWxTaWduIEVDQyBSb290IENBIC0gUjQxEzARBgNVBAoTCkdsb2Jh\n"
//"bFNpZ24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMTIxMTEzMDAwMDAwWhcNMzgw\n"
//"MTE5MDMxNDA3WjBQMSQwIgYDVQQLExtHbG9iYWxTaWduIEVDQyBSb290IENBIC0g\n"
//"UjQxEzARBgNVBAoTCkdsb2JhbFNpZ24xEzARBgNVBAMTCkdsb2JhbFNpZ24wWTAT\n"
//"BgcqhkjOPQIBBggqhkjOPQMBBwNCAAS4xnnTj2wlDp8uORkcA6SumuU5BwkWymOx\n"
//"uYb4ilfBV85C+nOh92VC/x7BALJucw7/xyHlGKSq2XE/qNS5zowdo0IwQDAOBgNV\n"
//"HQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUVLB7rUW44kB/\n"
//"+wpu+74zyTyjhNUwCgYIKoZIzj0EAwIDRwAwRAIgIk90crlgr/HmnKAWBVBfw147\n"
//"bmF0774BxL4YSFlhgjICICadVGNA3jdgUM/I2O2dgq43mLyjj0xMqTQrbO/7lZsm\n"
//"-----END CERTIFICATE-----";
//
//const char *root_cert_mosquitto =
//"-----BEGIN CERTIFICATE-----\n"
//"MIIEAzCCAuugAwIBAgIUBY1hlCGvdj4NhBXkZ/uLUZNILAwwDQYJKoZIhvcNAQEL\n"
//"BQAwgZAxCzAJBgNVBAYTAkdCMRcwFQYDVQQIDA5Vbml0ZWQgS2luZ2RvbTEOMAwG\n"
//"A1UEBwwFRGVyYnkxEjAQBgNVBAoMCU1vc3F1aXR0bzELMAkGA1UECwwCQ0ExFjAU\n"
//"BgNVBAMMDW1vc3F1aXR0by5vcmcxHzAdBgkqhkiG9w0BCQEWEHJvZ2VyQGF0Y2hv\n"
//"by5vcmcwHhcNMjAwNjA5MTEwNjM5WhcNMzAwNjA3MTEwNjM5WjCBkDELMAkGA1UE\n"
//"BhMCR0IxFzAVBgNVBAgMDlVuaXRlZCBLaW5nZG9tMQ4wDAYDVQQHDAVEZXJieTES\n"
//"MBAGA1UECgwJTW9zcXVpdHRvMQswCQYDVQQLDAJDQTEWMBQGA1UEAwwNbW9zcXVp\n"
//"dHRvLm9yZzEfMB0GCSqGSIb3DQEJARYQcm9nZXJAYXRjaG9vLm9yZzCCASIwDQYJ\n"
//"KoZIhvcNAQEBBQADggEPADCCAQoCggEBAME0HKmIzfTOwkKLT3THHe+ObdizamPg\n"
//"UZmD64Tf3zJdNeYGYn4CEXbyP6fy3tWc8S2boW6dzrH8SdFf9uo320GJA9B7U1FW\n"
//"Te3xda/Lm3JFfaHjkWw7jBwcauQZjpGINHapHRlpiCZsquAthOgxW9SgDgYlGzEA\n"
//"s06pkEFiMw+qDfLo/sxFKB6vQlFekMeCymjLCbNwPJyqyhFmPWwio/PDMruBTzPH\n"
//"3cioBnrJWKXc3OjXdLGFJOfj7pP0j/dr2LH72eSvv3PQQFl90CZPFhrCUcRHSSxo\n"
//"E6yjGOdnz7f6PveLIB574kQORwt8ePn0yidrTC1ictikED3nHYhMUOUCAwEAAaNT\n"
//"MFEwHQYDVR0OBBYEFPVV6xBUFPiGKDyo5V3+Hbh4N9YSMB8GA1UdIwQYMBaAFPVV\n"
//"6xBUFPiGKDyo5V3+Hbh4N9YSMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEL\n"
//"BQADggEBAGa9kS21N70ThM6/Hj9D7mbVxKLBjVWe2TPsGfbl3rEDfZ+OKRZ2j6AC\n"
//"6r7jb4TZO3dzF2p6dgbrlU71Y/4K0TdzIjRj3cQ3KSm41JvUQ0hZ/c04iGDg/xWf\n"
//"+pp58nfPAYwuerruPNWmlStWAXf0UTqRtg4hQDWBuUFDJTuWuuBvEXudz74eh/wK\n"
//"sMwfu1HFvjy5Z0iMDU8PUDepjVolOCue9ashlS4EB5IECdSR2TItnAIiIwimx839\n"
//"LdUdRudafMu5T5Xma182OC0/u/xRlEm+tvKGGmfFcN0piqVl8OrSPBgIlb+1IKJE\n"
//"m/XriWr/Cq4h/JfB7NTsezVslgkBaoU=\n"
//"-----END CERTIFICATE-----";

const char *client_cert_mosquitto =
"-----BEGIN CERTIFICATE-----\n"
"MIIDcjCCAlqgAwIBAgIBADANBgkqhkiG9w0BAQsFADCBkDELMAkGA1UEBhMCR0Ix\n"
"FzAVBgNVBAgMDlVuaXRlZCBLaW5nZG9tMQ4wDAYDVQQHDAVEZXJieTESMBAGA1UE\n"
"CgwJTW9zcXVpdHRvMQswCQYDVQQLDAJDQTEWMBQGA1UEAwwNbW9zcXVpdHRvLm9y\n"
"ZzEfMB0GCSqGSIb3DQEJARYQcm9nZXJAYXRjaG9vLm9yZzAeFw0yMTAzMjIxNjUx\n"
"MDdaFw0yMTA2MjAxNjUxMDdaMEwxCzAJBgNVBAYTAkZSMQ4wDAYDVQQIDAVQYXJp\n"
"czEQMA4GA1UECgwHU3dvb3BpbjEbMBkGA1UEAwwSdGVzdC5tb3NxdWl0dG8ub3Jn\n"
"MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAx2We23L1c0+/nNQwkrVp\n"
"eadnGdHqG4nrMjkN8LJvMB9if9PXxdqpjuDk9a/r7iiUZGmiMlGHxRQr3vD04CcF\n"
"EFwo+fSpYRcWEXQCXvbnMMpLAMv3ETbGO58DWclKFq/sDCqCgqMLFTMyJDeK+N8v\n"
"MZHk2y8aqWb6bg/7WQpLryofxT+ZQp9pmLm0Vetsau7btoXMIK/mzT3LTe0SZlh4\n"
"kGUcw6UGbtcAeSquizkh/8tDquR4sg+IfbBXIZRTz0oU40hNKzXOwPk/TRH/REUC\n"
"Qm/AsD5t9eyLo+mBiJ/3h684q+kabi2MGZZQ/GeJuesFocxw4xQRDg+hbOXCanUY\n"
"CQIDAQABoxowGDAJBgNVHRMEAjAAMAsGA1UdDwQEAwIF4DANBgkqhkiG9w0BAQsF\n"
"AAOCAQEAEVM0cuvcwsvnbvkivwfdzUqXpFvUYpcHgm00Qa5NahNjREmbnMLF4UUh\n"
"+ypOENxu3EElCpoZyrU9ZgsgmWp9uETqFZP/7tdgKnc8rOjRqknmO8pg546JXJHA\n"
"s+9DUPa+v3Q9kOrm/+JuFZ4CVaNswPGzGCh904rpd3cnysQAOGVDiF4BJ86k1aow\n"
"f+qRUFyr+ZOJUI7PqLnwdh1fEfyqoYkM/djaVFyipRC3F7EXnWRrFn0uJ26a5fzp\n"
"Me1S5Z+kUnGg9NVNduiOf+YdXTSkF/1hqromBFKHY93Y++ZpwmSdSqLxpXEdaGLP\n"
"Ag/ECbIulR2WzXKazSg1wjRumq6j9w==\n"
"-----END CERTIFICATE-----";

const char *client_key_mosquitto =
"-----BEGIN RSA PRIVATE KEY-----\n"
"MIIEpQIBAAKCAQEAx2We23L1c0+/nNQwkrVpeadnGdHqG4nrMjkN8LJvMB9if9PX\n"
"xdqpjuDk9a/r7iiUZGmiMlGHxRQr3vD04CcFEFwo+fSpYRcWEXQCXvbnMMpLAMv3\n"
"ETbGO58DWclKFq/sDCqCgqMLFTMyJDeK+N8vMZHk2y8aqWb6bg/7WQpLryofxT+Z\n"
"Qp9pmLm0Vetsau7btoXMIK/mzT3LTe0SZlh4kGUcw6UGbtcAeSquizkh/8tDquR4\n"
"sg+IfbBXIZRTz0oU40hNKzXOwPk/TRH/REUCQm/AsD5t9eyLo+mBiJ/3h684q+ka\n"
"bi2MGZZQ/GeJuesFocxw4xQRDg+hbOXCanUYCQIDAQABAoIBAQCgwpePSmIGNBfC\n"
"c3kt70E4qfT/5jQfO9T6SvGoS9RFMiiR8tQWWrM11LAEPW+0NddeFaSFeSJDjH3K\n"
"Yu6rAgqdJBCNZephsuFB9QwsUFgZCB/sZoWtxUGRtCRZcWZgYDmpnWRXuugmkX4t\n"
"oPJUsGw2RPAEnzeZuMLTleMJ6LkYYSpk0y6p/pSQHVAzzJko8fQUwZYscjT8/x6r\n"
"i5n1c3S6dRuR43fnpWjoIvkhcMjeZgjseWfkG/F8XwTdKfJqVTay6vRqHAXLdb9A\n"
"slbWs3Xu2/JtQfWN3poZSRDpfWMSXcHibb+UYYcu+00Aadc9Lt/H8Rik0rWTAoYa\n"
"JXMsGhP5AoGBAPWJVBoRJ3cVX7x4mE+7ipRyPinjWpJeVCFNpyrLGzCHFD5kXqJW\n"
"0Nk+3KCdx5lj7LCus92lCoTxTQfIG2PHYHQVpQHPElf4ImEdKTyp+nWZ5UpcxU/e\n"
"B9OARqKdlr6j1Vz9WFsJBW8J4N/KBiJtPfnjsof08WS0231mjozc2bu/AoGBAM/k\n"
"71Ot8LEQQRer4eYw2DvMzcSSMksk98Y5jJ/spdrei4ggWlCLMWWiLqcV6Mly1m2b\n"
"u+q+ZVCj9hQON0o5Wk7frDFqJez8UJcW6TfE4xEg9YLUe92WfhLFMR5z5WVFe79n\n"
"1InvT7CH66XpaSXC2EwedEualO0RvU6R/ruvKr43AoGAPmMiZ20sZemcZbsOmFPC\n"
"pm1Qtjv6a25kWgHtE8Z/phjsX94I3Z7JeimqzXTJnoPxRANf2FVzEsxtEikaLIQg\n"
"Ud6eCpOeyZNWyO4r6CoYZYEw7XAdAmob+vWA/RilkNL4B6SnqpnkM46DANnILT8s\n"
"nC3q0vi9zSTXEEnatpf3CukCgYEAslh1r9XCbEPDc6kyrCUu+24hAAdjOe/G17i7\n"
"GN8NEYxlKyyGDh67C7NsTk6PW3I23eUB+snHZLyj+GSRrg0xmgRsVh/31SxHfrZa\n"
"8ErpeOJLfCcI8/mMwlGEZ9mI664GERfjz0kLliekUrrAR3K11+cwYUOFizLyshqw\n"
"6xXEl1UCgYEAkBxSNft24vG9IohzTVsvt4qE8HOK7aKJN7Lh+GuY9h4eQWyFI/PY\n"
"v4OiH+C1LwaEmlrUHRUAT2DTLJeC0tv7+PxyxJuAtrrc8MqkUt6vQoRZKKc1lCSf\n"
"FXjlaszOEO4a04ouCCzEL/rrdzzuQ4TNtIP1J7odOIF4irLiHLF6X84=\n"
"-----END RSA PRIVATE KEY-----";

//const char *root_cert_primary =
//
//const char *root_cert_backup =

// In case we ever need extra topics
const int ex_num_topics = 0;
const char* ex_topics[ex_num_topics];
//const int ex_num_topics = 1;
//const char* ex_topics[ex_num_topics] = {
//  "/devices/my-device/tbd/#"
//};
