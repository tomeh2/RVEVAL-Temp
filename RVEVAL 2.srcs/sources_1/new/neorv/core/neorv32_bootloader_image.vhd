-- The NEORV32 RISC-V Processor: https://github.com/stnolting/neorv32
-- Auto-generated memory initialization file (for BOOTLOADER) from source file <bootloader/main.bin>
-- Size: 3984 bytes
-- MARCH: default
-- Built: 09.01.2023 19:30:14

-- prototype defined in 'neorv32_package.vhd'
package body neorv32_bootloader_image is

constant bootloader_init_image : mem32_t := (
x"30005073",
x"80010117",
x"1f810113",
x"80010197",
x"7f418193",
x"00000517",
x"0e050513",
x"30551073",
x"30001073",
x"30401073",
x"b0001073",
x"b8001073",
x"b0201073",
x"b8201073",
x"00000213",
x"00000293",
x"00000313",
x"00000393",
x"00000813",
x"00000893",
x"00000913",
x"00000993",
x"00000a13",
x"00000a93",
x"00000b13",
x"00000b93",
x"00000c13",
x"00000c93",
x"00000d13",
x"00000d93",
x"00000e13",
x"00000e93",
x"00000f13",
x"00000f93",
x"00001597",
x"f0858593",
x"80010617",
x"f7060613",
x"80010697",
x"f6868693",
x"00c58e63",
x"00d65c63",
x"0005a703",
x"00e62023",
x"00458593",
x"00460613",
x"fedff06f",
x"80010717",
x"f4470713",
x"80818793",
x"00f75863",
x"00072023",
x"00470713",
x"ff5ff06f",
x"00000513",
x"00000593",
x"060000ef",
x"30401073",
x"34051073",
x"10500073",
x"0000006f",
x"ff810113",
x"00812023",
x"00912223",
x"34202473",
x"02044663",
x"34102473",
x"00041483",
x"0034f493",
x"00240413",
x"34141073",
x"00300413",
x"00941863",
x"34102473",
x"00240413",
x"34141073",
x"00012403",
x"00412483",
x"00810113",
x"30200073",
x"fb010113",
x"04912223",
x"800004b7",
x"0004a223",
x"800007b7",
x"0007a023",
x"ffff07b7",
x"04112623",
x"04812423",
x"05212023",
x"03312e23",
x"03412c23",
x"03512a23",
x"03612823",
x"03712623",
x"03812423",
x"03912223",
x"03a12023",
x"01b12e23",
x"79878793",
x"30579073",
x"fe802783",
x"00080737",
x"00e7f7b3",
x"00078863",
x"fa002423",
x"40100793",
x"faf02423",
x"fe802783",
x"40000737",
x"00e7f7b3",
x"06078863",
x"f4002023",
x"f4002423",
x"000067b7",
x"f4002623",
x"20578793",
x"f4f02023",
x"f4002423",
x"f4002623",
x"f4002783",
x"00e7f7b3",
x"fe079ce3",
x"f4002783",
x"02000737",
x"00e7e7b3",
x"f4f02023",
x"f4002783",
x"08000737",
x"00e7e7b3",
x"f4f02023",
x"f4002783",
x"fe1fe737",
x"43f70713",
x"00e7f7b3",
x"00801737",
x"60070713",
x"00e7e7b3",
x"f4f02023",
x"fe802783",
x"00010737",
x"00e7f7b3",
x"00078863",
x"00100793",
x"fcf02423",
x"fc002623",
x"fa002023",
x"fe002683",
x"000097b7",
x"ffff7637",
x"00000713",
x"5ff78793",
x"a0060613",
x"1cd7ec63",
x"000016b7",
x"00000793",
x"ffe68693",
x"1ce6ee63",
x"fff70713",
x"01879793",
x"00e7e7b3",
x"10000737",
x"00e7e7b3",
x"faf02023",
x"fe802783",
x"00020737",
x"00e7f7b3",
x"02078063",
x"fe002783",
x"0027d793",
x"f8f02c23",
x"f8002e23",
x"08000793",
x"30479073",
x"30046073",
x"ffff1537",
x"d8c50513",
x"424000ef",
x"f1302573",
x"3a8000ef",
x"ffff1537",
x"dc450513",
x"410000ef",
x"fe402503",
x"394000ef",
x"ffff1537",
x"dcc50513",
x"3fc000ef",
x"fe002503",
x"380000ef",
x"ffff1537",
x"dd450513",
x"3e8000ef",
x"30102573",
x"36c000ef",
x"ffff1537",
x"ddc50513",
x"3d4000ef",
x"fc002573",
x"358000ef",
x"ffff1537",
x"de050513",
x"3c0000ef",
x"fe802503",
x"ffff1437",
x"340000ef",
x"ffff1537",
x"de850513",
x"3a8000ef",
x"ff802503",
x"32c000ef",
x"df040513",
x"398000ef",
x"ff002503",
x"31c000ef",
x"ffff1537",
x"dfc50513",
x"384000ef",
x"ffc02503",
x"308000ef",
x"df040513",
x"374000ef",
x"ff402503",
x"2f8000ef",
x"fe802783",
x"00020737",
x"00e7f7b3",
x"04078c63",
x"ffff1537",
x"e0450513",
x"350000ef",
x"2ac000ef",
x"fe002403",
x"000409b7",
x"00002a37",
x"00341413",
x"00a40933",
x"00893433",
x"00b40433",
x"fe802783",
x"0137f7b3",
x"0a078863",
x"fa002783",
x"0147f7b3",
x"0a079263",
x"ffff1537",
x"fa402783",
x"e3050513",
x"308000ef",
x"ffff19b7",
x"e3c98513",
x"2fc000ef",
x"06c00a13",
x"07800b93",
x"07300c13",
x"06500c93",
x"ffff17b7",
x"ebc78513",
x"2e0000ef",
x"fa402403",
x"fe045ee3",
x"0ff47413",
x"00040513",
x"240000ef",
x"ffff17b7",
x"d8878513",
x"2c0000ef",
x"07200793",
x"06f41863",
x"ffff02b7",
x"00028067",
x"00170713",
x"01071713",
x"00c686b3",
x"01075713",
x"e19ff06f",
x"ffe78613",
x"0fd67613",
x"00061a63",
x"00375713",
x"00178793",
x"0ff7f793",
x"e0dff06f",
x"00175713",
x"ff1ff06f",
x"1d4000ef",
x"f485e2e3",
x"00b41463",
x"f3256ee3",
x"00100513",
x"658000ef",
x"ffff1537",
x"d8850513",
x"254000ef",
x"00000513",
x"039000ef",
x"17440c63",
x"028a6463",
x"17940c63",
x"06800793",
x"e3c98513",
x"02f40c63",
x"03f00793",
x"18f40063",
x"ffff1537",
x"f6050513",
x"0240006f",
x"07500793",
x"02f40263",
x"17740063",
x"ff8414e3",
x"0044a403",
x"02041063",
x"ffff1537",
x"ec450513",
x"1fc000ef",
x"f11ff06f",
x"00000513",
x"5e8000ef",
x"f05ff06f",
x"ffff1537",
x"ee050513",
x"1e0000ef",
x"00040513",
x"164000ef",
x"ffff1537",
x"ee850513",
x"1cc000ef",
x"00400537",
x"150000ef",
x"ffff1537",
x"f0050513",
x"1b8000ef",
x"fa402903",
x"fe095ee3",
x"0ff97913",
x"00090513",
x"118000ef",
x"07900793",
x"eaf91ae3",
x"54c000ef",
x"00050663",
x"00300513",
x"1e4000ef",
x"ffff1537",
x"f0c50513",
x"180000ef",
x"01045b13",
x"00400937",
x"00010db7",
x"fff00d13",
x"4cc000ef",
x"3fc000ef",
x"0d800513",
x"3a4000ef",
x"00090513",
x"3b0000ef",
x"384000ef",
x"4d0000ef",
x"00157a93",
x"fe0a9ce3",
x"fffb0b13",
x"01b90933",
x"fdab18e3",
x"ff002683",
x"00400937",
x"00000d13",
x"00c90d93",
x"00dd0733",
x"00072583",
x"01bd0533",
x"00d12623",
x"00ba8ab3",
x"004d0d13",
x"668000ef",
x"00c12683",
x"fe8d60e3",
x"4788d5b7",
x"afe58593",
x"00400537",
x"650000ef",
x"00040593",
x"00490513",
x"644000ef",
x"00890513",
x"415005b3",
x"638000ef",
x"ffff1537",
x"d7050513",
x"ed9ff06f",
x"00100513",
x"eddff06f",
x"0044a783",
x"e6079ae3",
x"ffff1537",
x"f1c50513",
x"ebdff06f",
x"00100513",
x"e65ff06f",
x"ffff1537",
x"f2c50513",
x"ea9ff06f",
x"f9402583",
x"f9002503",
x"f9402783",
x"fef59ae3",
x"00008067",
x"00040737",
x"fa002783",
x"00e7f7b3",
x"fe079ce3",
x"faa02223",
x"00008067",
x"fe010113",
x"01212823",
x"00050913",
x"03000513",
x"00112e23",
x"00812c23",
x"00912a23",
x"01312623",
x"fc9ff0ef",
x"07800513",
x"ffff14b7",
x"fbdff0ef",
x"01c00413",
x"f6c48493",
x"ffc00993",
x"008957b3",
x"00f7f793",
x"00f487b3",
x"0007c503",
x"ffc40413",
x"f99ff0ef",
x"ff3414e3",
x"01c12083",
x"01812403",
x"01412483",
x"01012903",
x"00c12983",
x"02010113",
x"00008067",
x"ff010113",
x"00812423",
x"01212023",
x"00112623",
x"00912223",
x"00050413",
x"00a00913",
x"00044483",
x"00140413",
x"00049e63",
x"00c12083",
x"00812403",
x"00412483",
x"00012903",
x"01010113",
x"00008067",
x"01249663",
x"00d00513",
x"f2dff0ef",
x"00048513",
x"f25ff0ef",
x"fc9ff06f",
x"ff010113",
x"00812423",
x"00050413",
x"ffff1537",
x"d2850513",
x"00112623",
x"f91ff0ef",
x"00241793",
x"ffff1537",
x"008787b3",
x"f7c50513",
x"00f50533",
x"f79ff0ef",
x"30047073",
x"fe802783",
x"00010737",
x"00e7f7b3",
x"00078863",
x"00100793",
x"fcf02423",
x"fc002623",
x"0000006f",
x"fb010113",
x"04112623",
x"04512423",
x"04612223",
x"04712023",
x"02812e23",
x"02912c23",
x"02a12a23",
x"02b12823",
x"02c12623",
x"02d12423",
x"02e12223",
x"02f12023",
x"01012e23",
x"01112c23",
x"01c12a23",
x"01d12823",
x"01e12623",
x"01f12423",
x"342024f3",
x"800007b7",
x"00778793",
x"0af49663",
x"fe802783",
x"00010737",
x"00e7f7b3",
x"00078863",
x"fc802783",
x"0017c793",
x"fcf02423",
x"fe802783",
x"00020737",
x"00e7f7b3",
x"02078863",
x"e29ff0ef",
x"fe002783",
x"fff00713",
x"f8e02c23",
x"0027d793",
x"00a78533",
x"00f537b3",
x"00b787b3",
x"f8f02e23",
x"f8a02c23",
x"00000013",
x"03c12403",
x"04c12083",
x"04812283",
x"04412303",
x"04012383",
x"03812483",
x"03412503",
x"03012583",
x"02c12603",
x"02812683",
x"02412703",
x"02012783",
x"01c12803",
x"01812883",
x"01412e03",
x"01012e83",
x"00c12f03",
x"00812f83",
x"05010113",
x"30200073",
x"00700793",
x"00f49c63",
x"800007b7",
x"0007a783",
x"00078663",
x"00100513",
x"e8dff0ef",
x"34102473",
x"fe802783",
x"00040737",
x"00e7f7b3",
x"04078263",
x"ffff1537",
x"d3050513",
x"e15ff0ef",
x"00048513",
x"d99ff0ef",
x"02000513",
x"d79ff0ef",
x"00040513",
x"d89ff0ef",
x"02000513",
x"d69ff0ef",
x"34302573",
x"d79ff0ef",
x"ffff1537",
x"d8850513",
x"de1ff0ef",
x"00440413",
x"34141073",
x"f39ff06f",
x"fa800713",
x"00072783",
x"eff7f793",
x"00f72023",
x"00008067",
x"faa02623",
x"fa802783",
x"fe07cee3",
x"fac02503",
x"00008067",
x"ff010113",
x"00812423",
x"00050413",
x"01055513",
x"0ff57513",
x"00112623",
x"fd5ff0ef",
x"00845513",
x"0ff57513",
x"fc9ff0ef",
x"0ff47513",
x"00812403",
x"00c12083",
x"01010113",
x"fb5ff06f",
x"fa800713",
x"00072783",
x"e1f7f793",
x"1007e793",
x"00f72023",
x"00008067",
x"fd010113",
x"02812423",
x"03212023",
x"01312e23",
x"01412c23",
x"02112623",
x"02912223",
x"00050993",
x"00058913",
x"00000413",
x"00400a13",
x"04099463",
x"00400713",
x"fa402783",
x"fe07dee3",
x"00c10693",
x"008686b3",
x"00f68023",
x"00140413",
x"fee414e3",
x"02c12083",
x"02812403",
x"00c12503",
x"02412483",
x"02012903",
x"01c12983",
x"01812a03",
x"03010113",
x"00008067",
x"f75ff0ef",
x"00300513",
x"012404b3",
x"f19ff0ef",
x"00048513",
x"f25ff0ef",
x"00000513",
x"f09ff0ef",
x"00050493",
x"eedff0ef",
x"00c10793",
x"008787b3",
x"00978023",
x"00140413",
x"f94410e3",
x"fa1ff06f",
x"ff010113",
x"00112623",
x"f2dff0ef",
x"00600513",
x"ed5ff0ef",
x"00c12083",
x"01010113",
x"eb5ff06f",
x"fe010113",
x"00112e23",
x"f0dff0ef",
x"00500513",
x"eb5ff0ef",
x"00000513",
x"eadff0ef",
x"00a12623",
x"e91ff0ef",
x"01c12083",
x"00c12503",
x"02010113",
x"00008067",
x"ff010113",
x"00112623",
x"fa5ff0ef",
x"fc1ff0ef",
x"00257793",
x"fff00513",
x"02078063",
x"ec5ff0ef",
x"00400513",
x"e6dff0ef",
x"e55ff0ef",
x"fa1ff0ef",
x"01e51513",
x"41f55513",
x"00c12083",
x"01010113",
x"00008067",
x"fd010113",
x"01612823",
x"00100793",
x"80000b37",
x"02812423",
x"02112623",
x"02912223",
x"03212023",
x"01312e23",
x"01412c23",
x"01512a23",
x"01712623",
x"01812423",
x"00fb2023",
x"00050413",
x"02051863",
x"ffff1537",
x"d3c50513",
x"bc1ff0ef",
x"004005b7",
x"00040513",
x"e61ff0ef",
x"4788d7b7",
x"afe78793",
x"04f50863",
x"00000513",
x"0380006f",
x"ffff1537",
x"d5c50513",
x"b95ff0ef",
x"00400537",
x"b19ff0ef",
x"ffff1537",
x"d6850513",
x"b81ff0ef",
x"fe802783",
x"00080737",
x"00e7f7b3",
x"00079663",
x"00300513",
x"bc1ff0ef",
x"f19ff0ef",
x"fa0502e3",
x"ff1ff06f",
x"004009b7",
x"00498593",
x"00040513",
x"df9ff0ef",
x"00050a13",
x"00898593",
x"00040513",
x"de9ff0ef",
x"ff002c03",
x"00050a93",
x"ffca7b93",
x"00000913",
x"00000493",
x"00c98993",
x"013905b3",
x"05791c63",
x"015484b3",
x"00200513",
x"fa0494e3",
x"ffff1537",
x"d7050513",
x"b05ff0ef",
x"02c12083",
x"02812403",
x"800007b7",
x"0147a223",
x"000b2023",
x"02412483",
x"02012903",
x"01c12983",
x"01812a03",
x"01412a83",
x"01012b03",
x"00c12b83",
x"00812c03",
x"03010113",
x"00008067",
x"00040513",
x"d6dff0ef",
x"012c07b3",
x"00a484b3",
x"00a7a023",
x"00490913",
x"f8dff06f",
x"fd010113",
x"02812423",
x"02912223",
x"01312e23",
x"02112623",
x"03212023",
x"01412c23",
x"00050493",
x"00b12623",
x"00000413",
x"00400993",
x"00c10793",
x"008787b3",
x"0007ca03",
x"dd1ff0ef",
x"d01ff0ef",
x"00200513",
x"ca9ff0ef",
x"00848933",
x"00090513",
x"cb1ff0ef",
x"000a0513",
x"c95ff0ef",
x"c7dff0ef",
x"dc9ff0ef",
x"00157513",
x"fe051ce3",
x"00140413",
x"fb341ee3",
x"02c12083",
x"02812403",
x"02412483",
x"02012903",
x"01c12983",
x"01812a03",
x"03010113",
x"00008067",
x"ff010113",
x"00112623",
x"00812423",
x"30047073",
x"ff002403",
x"00050463",
x"40400437",
x"ffff1537",
x"d7450513",
x"9f1ff0ef",
x"00040513",
x"975ff0ef",
x"ffff1537",
x"d8450513",
x"9ddff0ef",
x"00010737",
x"fa002783",
x"fe07cee3",
x"00e7f7b3",
x"fe078ae3",
x"000400e7",
x"52450a07",
x"00005f52",
x"5252450a",
x"4358455f",
x"00000020",
x"69617741",
x"676e6974",
x"6f656e20",
x"32337672",
x"6578655f",
x"6e69622e",
x"202e2e2e",
x"00000000",
x"64616f4c",
x"20676e69",
x"00004028",
x"2e2e2e29",
x"0000000a",
x"00004b4f",
x"746f6f42",
x"20676e69",
x"6d6f7266",
x"00000020",
x"0a2e2e2e",
x"0000000a",
x"3c0a0a0a",
x"454e203c",
x"3356524f",
x"6f422032",
x"6f6c746f",
x"72656461",
x"0a3e3e20",
x"444c420a",
x"4a203a56",
x"20206e61",
x"30322039",
x"480a3332",
x"203a5657",
x"00000020",
x"4449430a",
x"0020203a",
x"4b4c430a",
x"0020203a",
x"4153490a",
x"0020203a",
x"00202b20",
x"434f530a",
x"0020203a",
x"454d490a",
x"00203a4d",
x"74796220",
x"40207365",
x"00000000",
x"454d440a",
x"00203a4d",
x"75410a0a",
x"6f626f74",
x"6920746f",
x"7338206e",
x"7250202e",
x"20737365",
x"20796e61",
x"2079656b",
x"61206f74",
x"74726f62",
x"00000a2e",
x"726f6241",
x"2e646574",
x"00000a0a",
x"69617641",
x"6c62616c",
x"4d432065",
x"0a3a7344",
x"203a6820",
x"706c6548",
x"3a72200a",
x"73655220",
x"74726174",
x"3a75200a",
x"6c705520",
x"0a64616f",
x"203a7320",
x"726f7453",
x"6f742065",
x"616c6620",
x"200a6873",
x"4c203a6c",
x"2064616f",
x"6d6f7266",
x"616c6620",
x"200a6873",
x"42203a78",
x"20746f6f",
x"6d6f7266",
x"616c6620",
x"28206873",
x"29504958",
x"3a65200a",
x"65784520",
x"65747563",
x"00000000",
x"444d430a",
x"00203e3a",
x"65206f4e",
x"75636578",
x"6c626174",
x"76612065",
x"616c6961",
x"2e656c62",
x"00000000",
x"74697257",
x"00002065",
x"74796220",
x"74207365",
x"5053206f",
x"6c662049",
x"20687361",
x"00002040",
x"7928203f",
x"20296e2f",
x"00000000",
x"616c460a",
x"6e696873",
x"2e2e2e67",
x"00000020",
x"65206f4e",
x"75636578",
x"6c626174",
x"00002e65",
x"20296328",
x"53207962",
x"68706574",
x"4e206e61",
x"69746c6f",
x"670a676e",
x"75687469",
x"6f632e62",
x"74732f6d",
x"746c6f6e",
x"2f676e69",
x"726f656e",
x"00323376",
x"61766e49",
x"2064696c",
x"00444d43",
x"33323130",
x"37363534",
x"62613938",
x"66656463",
x"00455845",
x"5a495300",
x"48430045",
x"4600534b",
x"0048534c"
);

end neorv32_bootloader_image;
