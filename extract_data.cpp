#include <iostream>
#include <fstream>
// #include <yaml-cpp/yaml.h>

#include "extract_data.h"

Data::Data(){

        hip         << 0.03133385093592258, 0.031331147345075416, 0.031321960589438425, 0.03130629556672405, 0.031284173841463855, 0.03125563302303276, 0.031220719335986767, 0.031179491317643776, 0.03113202047971376, 0.031078391012701063, 0.031018699422330764, 0.03095305411151567, 0.03088157491256089, 0.0308043925734772, 0.030721648202576075, 0.03063349267578928, 0.03054008601137035, 0.030441596716794042, 0.0303382011127688, 0.03023008263932322, 0.03011743114891358, 0.03000044219143503, 0.02987931629590781, 0.02975425825344784, 0.029625476405924597, 0.029493181944478357, 0.029357588221794816, 0.02921891008173681, 0.029077363209617176, 0.02893316350606583, 0.028786526487096186, 0.028637666712626596, 0.028486797245363966, 0.028334129141609442, 0.028179870975207525, 0.028024228395531325, 0.027867403720084345, 0.027709595562002736, 0.027550998492463296, 0.027391802737746936, 0.02723219391047276, 0.027072352774304163, 0.026912455041242837, 0.0267526712004537, 0.02659316637743704, 0.026434100222220133, 0.026275626825278037, 0.026117894661090973, 0.025961046558378928, 0.02580521968073231, 0.025650545540754126, 0.025497150031701502, 0.025345153476538325, 0.025194670693502078, 0.02504581107614711, 0.024898678686977278, 0.024753372364343933, 0.02460998583935793, 0.024468607861919945, 0.024329322334707037, 0.02419220845376638, 0.02405734085422214, 0.023924789761425588, 0.023794621145818394, 0.023666896879731125, 0.02354167489579082, 0.02341900934584218, 0.023298950758751654, 0.02318154620263819, 0.023066839443540797, 0.022954871099475315, 0.022845678794110823, 0.022739297308264156, 0.022635758728480327, 0.022535092592393974, 0.02243732603062196, 0.022342483904989018, 0.022250588942931988, 0.022161678074472616, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.0231, 0.028084295271906153, 0.028226063917243385, 0.028362201917387194, 0.028499846184900528, 0.02864049947309648, 0.028784138860782498, 0.02893071793868953, 0.029080184264219246, 0.02923248116638115, 0.029387547594481457, 0.02954531793033044, 0.029705721797651224, 0.029868683872144176, 0.030034123693162322, 0.03020195547743618, 0.030372087933797504, 0.030544424086845656, 0.030718861110282325, 0.030895290160449368, 0.031073596220285736, 0.031253657953980295, 0.03143534757348011, 0.031618530719506814, 0.031803066356733536, 0.031988806685052266, 0.032175597070162626, 0.032363275994937796, 0.03255167503336784, 0.03274061884926634, 0.032929925222261534, 0.033119405100221336, 0.03330886268226403, 0.033498095534625016, 0.03368689474054019, 0.03387504508687375, 0.034062325291210754, 0.03424850825144091, 0.03443336135181525, 0.03461664680750936, 0.034798122049080046, 0.03497754014919077, 0.03515465029206303, 0.03532919828561505, 0.035500927115921364, 0.03566957754325521, 0.03583488873859329, 0.035996598959063326, 0.036154446260396746, 0.03630816924402375, 0.036457507836014855, 0.03660220409464711, 0.0367420030429447, 0.03687665352213371, 0.037005909061552406, 0.037129528760198455, 0.037247278174749925, 0.037358930208603675, 0.03746426599621954, 0.03756307577685957, 0.037655159751662795, 0.03774032891791306, 0.03781840587433876, 0.037889225591326174, 0.037952636140052155, 0.03800849937473547, 0.038056691562464044, 0.03809710395538886, 0.03812964330047548, 0.03815423228246843, 0.0381708098962417, 0.03817933174528605, 0.03817977026369911;
        hip_vel     << 0.00026771137211373773, -0.0029725727303659237, -0.00621403594734813, -0.009449001831739603, -0.01266883792744531, -0.01586757909441683, -0.019040332080538437, -0.022180552047004965, -0.025281819686019755, -0.02833788190726435, -0.03134268157734755, -0.034290383324041426, -0.03717539747420919, -0.03999240197972134, -0.04273636218526617, -0.04540254831646413, -0.04798655059484253, -0.05048429191539848, -0.0528920380505093, -0.05520640537225735, -0.05742436611337979, -0.05954325121075713, -0.061560750799763114, -0.06347491245281457, -0.06528413727142358, -0.0669871739579531, -0.06858311101132562, -0.07007136720048429, -0.07145168047731827, -0.07272409549932908, -0.07388894993716982, -0.07494685974160496, -0.07589870354419104, -0.07674560636348098, -0.0774889227838463, -0.07813021976743503, -0.07867125925146022, -0.0791139806749393, -0.07946048356856353, -0.07971301033006778, -0.07987392929762603, -0.0799457182204523, -0.07993094821737325, -0.07983226829662972, -0.07965239050521988, -0.0793940757665265, -0.07906012019873658, -0.0786533406255532, -0.07817656650938415, -0.07763262890935584, -0.0770243466697549, -0.07635451852653063, -0.07562591440338906, -0.07484126768950256, -0.07400326830977115, -0.07311455590041516, -0.07217771397327356, -0.07119526522785927, -0.07016966725348107, -0.06910330886857986, -0.06799850726822167, -0.0668575053779076, -0.06568246946931135, -0.06447548781938446, -0.06323856986132866, -0.06197364553722648, -0.06068256579672565, -0.05936710149041737, -0.05802894284202649, -0.05666970207005345, -0.055290914537506494, -0.05389403919040235, -0.05248045992822215, -0.051051487141865436, -0.049608359389959215, -0.04815224518675856, -0.04668424487796109, -0.045205392582497055, -0.04368436957465785, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07059531873395022, 0.06953096735664306, 0.06807382519250554, 0.0695723442700798, 0.07107393376107064, 0.0725561079507522, 0.07401311736335614, 0.0754428585517137, 0.07684318559884033, 0.07821186249492099, 0.07954655796688662, 0.080844845529807, 0.08210420405726263, 0.08332201882703123, 0.08449558198035387, 0.08562209483412266, 0.0866986733577013, 0.08772235019613733, 0.08869007689317958, 0.08959872965516373, 0.0904451147925366, 0.0912259753939328, 0.09193799853827246, 0.09257782269267813, 0.09314204659867877, 0.09362723935596656, 0.09402995122742906, 0.09434672542519666, 0.09457411126471854, 0.09470867753292743, 0.09474702659410503, 0.0946858103996206, 0.09452174721347252, 0.09425163900899929, 0.09387239190244831, 0.09338103050942229, 0.09277471758254976, 0.09205077937569349, 0.0912067240835292, 0.09024026260922913, 0.08914932987888903, 0.08793210622166017, 0.08658703865018436, 0.08511286186739642, 0.08350861880977263, 0.0817736805321898, 0.07990776522953999, 0.07791095618398458, 0.07578371842261618, 0.07352691387055207, 0.07114181478543365, 0.06863011526310829, 0.0659939406115815, 0.0632358544034987, 0.060358863030514014, 0.05736641759909463, 0.05426241303052906, 0.051051184250936334, 0.047737499384016197, 0.04432654988786587, 0.040823937610755236, 0.037235658771188035, 0.03356808490354615, 0.029827940849533333, 0.026022279907960095, 0.022158456290636366, 0.018244095068280307, 0.014287059823135245, 0.01029541825403852, 0.006277406008409793, 0.0022413890401644914, -0.0018041751879389325;
        hip_f       << 0.03211089310478989, 0.03210495038314982, 0.03209252411650021, 0.03207362355110432, 0.03204826623160969, 0.032016481934527774, 0.03197831351571267, 0.031933816774830176, 0.031883060249256155, 0.03182612494968256, 0.031763104040749904, 0.03169410246924169, 0.031619236542773876, 0.03153863346229537, 0.03145243081205633, 0.03136077601099809, 0.03126382572976405, 0.031161745277729327, 0.031054707964588705, 0.030942894441133652, 0.030826492023891507, 0.03070569400828655, 0.030580698974917216, 0.03045171009344552, 0.030318934428441895, 0.030182582251339414, 0.03004286636243133, 0.029900001426596826, 0.029754203326158895, 0.029605688533981035, 0.02945467350959922, 0.029301374120862093, 0.029146005093224455, 0.028988779488508393, 0.028829908214620583, 0.028669599567394063, 0.028508058805410744, 0.028345487758364655, 0.02818208446924351, 0.028018042870339592, 0.027853552492858407, 0.027688798209660408, 0.027523960010483656, 0.02735921280878737, 0.027194726279341812, 0.02703066472693086, 0.026867186985582922, 0.02670444633117651, 0.02654259043229667, 0.026381761322678398, 0.026222095395276913, 0.02606372341713429, 0.025906770562961152, 0.025751356466540854, 0.02559759528965288, 0.025445595805059344, 0.025295461492555307, 0.02514729064678558, 0.025001176495311635, 0.02485720732523498, 0.02471546661861901, 0.02457603319476284, 0.024438981357298887, 0.024304381045614526, 0.02417229798928135, 0.024042793863590522, 0.023915926451953427, 0.023791749806590632, 0.023670314406172385, 0.023551667314711, 0.023435852339678577, 0.02332291018843223, 0.023212878622479102, 0.02310579260917621, 0.02300168447051985, 0.02290058402873111, 0.02280253596038327, 0.022707683251382817;
        hip_f_vel   << -0.001351201222425245, -0.004592512256807189, -0.007832846788746779, -0.011066347885230922, -0.014288239501690976, -0.017491964618337005, -0.02067101126993113, -0.023818954221030735, -0.02692948630864077, -0.029996446504610856, -0.033013846800209926, -0.03597589772400717, -0.038877032294575745, -0.041711928226381374, -0.044475528229017465, -0.0471630582641273, -0.04977004364872019, -0.052292322919679904, -0.054726059401660414, -0.057067750444701866, -0.059314234324139105, -0.06146269482362653, -0.0635106635419872, -0.06545601998646174, -0.06729698953901202, -0.0690321393982133, -0.070660372613963, -0.07218092034808335, -0.07359333250615217, -0.07489746689249606, -0.07609347704719624, -0.0771817989283572, -0.07816313660468294, -0.07903844712285077, -0.07980892471130256, -0.08047598447895897, -0.08104124576117615, -0.08150651525754045, -0.08187377009932896, -0.0821451409733478, -0.08232289542240714, -0.08240942142593327, -0.08240721136028359, -0.08231884642907901, -0.08214698136721107, -0.08189432804883305, -0.08156364578302872, -0.08115772834762443, -0.0806793878955606, -0.08013144502889905, -0.07951671795540148, -0.07883801263413534, -0.07809811373150048, -0.07729977566604171, -0.07644571469245694, -0.07553860221862664, -0.07458105856571637, -0.07357564744115025, -0.0725248713183586, -0.07143116709045685, -0.07029690205539768, -0.06912437107096739, -0.06791579430695795, -0.06667331528750649, -0.06539900023286951, -0.06409483577324449, -0.06276272732572669, -0.06140450089407451, -0.06002190341855891, -0.05861660244387019, -0.05719018683680669, -0.05574416775471694, -0.054279979831802085, -0.052798982552572274, -0.05130246178476172, -0.04979163144562974, -0.04823325734234929, -0.046598785014533024;

        thigh       << 1.2001913274441254, 1.2000989784829188, 1.1997850631412705, 1.1992493717732169, 1.1984919986593947, 1.1975133235731565, 1.1963137612129173, 1.1948938926031634, 1.193254494456157, 1.1913965372855966, 1.1893211826247065, 1.1870297797310574, 1.1845238618480574, 1.181805142055167, 1.1788755087410616, 1.1757370207365083, 1.1723919021458755, 1.1688425369179887, 1.165091463198327, 1.1611413675055653, 1.1569950787758823, 1.1526555623185508, 1.1481259137261548, 1.1434093527820446, 1.1385092174064797, 1.1334289576818803, 1.1281721299958933, 1.1227423913390988, 1.1171434937922295, 1.1113792792356953, 1.1054536743117387, 1.099370685667138, 1.0931343955019854, 1.0867489574475626, 1.0802185927938852, 1.0735475870850304, 1.0667402870980423, 1.0598010982189496, 1.0527344822272136, 1.0455449554979284, 1.03823708762919, 1.030815500500198, 1.0232848677641524, 1.0156499147781652, 1.0079154189717745, 1.0000862106532626, 0.9921671742585964, 0.9841632501127414, 0.9760794367392082, 0.9679207929511373, 0.959692440971441, 0.9513995698615341, 0.943047439323395, 0.934641383901823, 0.926186817550315, 0.917689238583622, 0.9091542350737061, 0.9005874905737077, 0.8919947901785856, 0.8833820269113667, 0.8747552084056661, 0.8661204638364032, 0.8574840511581978, 0.8488523645759706, 0.8402319421517669, 0.8316294735324175, 0.8230518077201602, 0.8145059607525507, 0.8059991236573856, 0.797538670088693, 0.7891321634817754, 0.7807873639455579, 0.7725122346661933, 0.7643149476446648, 0.7562038885984802, 0.7481876608377487, 0.7402750879052885, 0.7324752147489197, 0.7247987183949821, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8086710222939685, 0.8171195938180429, 0.8250949713975999, 0.8330266542437819, 0.8409997102435802, 0.8490093290716099, 0.8570496523674157, 0.8651148041201685, 0.8731990004355337, 0.881296547257626, 0.8894018360117067, 0.8975093390794638, 0.9056136052910155, 0.9137092554707966, 0.9217909780419659, 0.9298535246202124, 0.9378917059496521, 0.9459003881579382, 0.9538744888495998, 0.961808973481361, 0.9696988519751497, 0.9775391755647224, 0.9853250339328086, 0.9930515525620893, 1.0007138903236332, 1.008307237374781, 1.0158268133584933, 1.0232678659098415, 1.030625669489327, 1.0378955245735344, 1.045072757104193, 1.052152718294237, 1.0591307848119342, 1.0660023593242325, 1.0727628714413384, 1.0794077791389685, 1.0859325699759301, 1.0923327632830127, 1.0986039126260578, 1.104741608568311, 1.1107414817940529, 1.1165992065938655, 1.1223105047032593, 1.1278711494844935, 1.1332769704387609, 1.1385238580334927, 1.1436077688270965, 1.1485247308707918, 1.1532708493646504, 1.1578423125423, 1.1622353977563782, 1.166446477734258, 1.1704720269713262, 1.1743086282267956, 1.1779529790853236, 1.1814018985457473, 1.184652333596925, 1.1877013657394655, 1.190546217411397, 1.193184258275293, 1.1956130113243466, 1.1978301587652465, 1.1998335476362765, 1.2016211951203737, 1.20319129351454, 1.204542214818896, 1.2056725149111167, 1.2065809372748117, 1.207266416253626, 1.2077280798062384, 1.2079652517413457, 1.2079774534156942;
        thigh_vel   << 0.009143692445235933, -0.101545282464628, -0.21239645699126328, -0.3232806505649977, -0.4340351213777588, -0.5445866021409542, -0.6548965452678192, -0.7648669383551824, -0.8744003202338294, -0.9834004393697552, -1.0917725755568468, -1.19942376962727, -1.3062630372334565, -1.4122015661435154, -1.5171528957691014, -1.6210330777388249, -1.723760816527359, -1.8252575893748744, -1.9254477449208296, -2.02425858019322, -2.1216203958387685, -2.2174665296057876, -2.311733368294314, -2.4043603386805423, -2.495289877952008, -2.5844673843226165, -2.6718411487581806, -2.7573622687684547, -2.8409845452618567, -2.9226643636076863, -3.002360560142385, -3.0800342753100454, -3.1556487946638585, -3.2291693789711537, -3.300563084653143, -3.3697985757572466, -3.43684592859977, -3.5016764302081342, -3.5642623716172013, -3.6245768369843296, -3.682593489481709, -3.738286354775204, -3.791629602977061, -3.8425973296223517, -3.8911633364447376, -3.9373009127659664, -3.9809826053659814, -4.02217990780121, -4.060863234615442, -4.097001721990363, -4.130562835538802, -4.161512242721294, -4.189813607896882, -4.215428402282143, -4.238315721109415, -4.258432069919217, -4.275731169712447, -4.290163795795021, -4.301677610012946, -4.310217003338661, -4.315722963693125, -4.318132936336497, -4.31738068176129, -4.313396186735801, -4.306105601605789, -4.295431190155747, -4.28129136392697, -4.263600618206977, -4.242269516255296, -4.21720492813038, -4.188310210342873, -4.155485376654591, -4.118627391659855, -4.077630544598004, -4.032386912978796, -3.982786926152057, -3.9287200394063144, -3.8700755293143345, -3.8039198911115046, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4.244434506163333, 4.107316170951211, 3.955118162483628, 3.976369310755945, 3.9959954677580187, 4.012829415373545, 4.026703451478323, 4.037662645169023, 4.045752827984196, 4.051017867327247, 4.053499373044167, 4.0532366930603745, 4.050266931935376, 4.044624985826286, 4.036343539681132, 4.02545314447318, 4.011982422191813, 3.9959580815627715, 3.9774049306457604, 3.9563460296269852, 3.932802799950476, 3.906795159617291, 3.878341651026618, 3.8474595470141315, 3.814164989652747, 3.7784731456045018, 3.740398356996657, 3.699954298148206, 3.657154152384284, 3.6120107630162117, 3.564536781806614, 3.5147448585472274, 3.462647825347029, 3.4082588759759256, 3.3515918265599756, 3.2926611306909264, 3.23148207209222, 3.1680711303986846, 3.102446082676905, 3.034626176351758, 2.964632311588917, 2.892487220235172, 2.818215640142541, 2.7418444837709837, 2.663402999759575, 2.5829229262778317, 2.5004386348745555, 2.41598726352152, 2.3296088375397606, 2.2413463771593682, 2.1512459904763492, 2.0593569505995415, 1.9657317558399574, 1.8704261719512003, 1.7734992554990412, 1.6750133575238593, 1.5750341068731755, 1.4736303727130429, 1.3708742059038592, 1.2668407590888549, 1.1616081855952305, 1.0552575173652892, 0.9478725223797675, 0.8395395423407084, 0.7303473114917978, 0.6203867576560353, 0.5097507868182971, 0.3985340527681386, 0.2868327134510394, 0.17474417586264515, 0.06236683146404606, -0.05020021581499522;
        thigh_f     << 1.2014783340136936, 1.2012804852928698, 1.200866553821685, 1.2002363629592048, 1.199389759579326, 1.1983267464747829, 1.1970475136009904, 1.1955524380713374, 1.1938420832555232, 1.191917197347542, 1.189778711459666, 1.187427737261308, 1.1848655641844208, 1.1820936562203945, 1.1791136483364506, 1.175927342542326, 1.1725367036404246, 1.168943854694797, 1.165151072255954, 1.1611607813798868, 1.1569755504807664, 1.1525980860573555, 1.1480312273332296, 1.1432779408511091, 1.1383413150610282, 1.1332245549412703, 1.127930976690046, 1.1224640025247181, 1.1168271556237166, 1.1110240552445838, 1.1050584120497948, 1.0989340236699896, 1.0926547705321878, 1.0862246119783867, 1.0796475826978216, 1.0729277894940081, 1.0660694084055005, 1.0590766821972588, 1.0519539182375177, 1.0447054867730294, 1.037335819613888, 1.0298494092370618, 1.0222508083169224, 1.0145446296884213, 1.0067355467541181, 0.9988282944127755, 0.9908276705541939, 0.9827385373439231, 0.9745658235640491, 0.9663145272944584, 0.9579897190038095, 0.949596545082767, 0.9411402317883795, 0.932626089629082, 0.9240595182555269, 0.9154460117490391, 0.9067911643242254, 0.8981006764436474, 0.889380361325024, 0.880636151803272, 0.8718741076200552, 0.8631004230797045, 0.8543214349899991, 0.8455436308886347, 0.8367736574952327, 0.8280183292732196, 0.8192846374943059, 0.8105797592353232, 0.8019110661648561, 0.7932861333639503, 0.7847127479835917, 0.7761989175895855, 0.7677528780524718, 0.7593831008194496, 0.7510982993827561, 0.742907434734309, 0.734821151775407, 0.7268589650078967;
        thigh_f_vel << -0.04497698166227071, -0.1529240435762182, -0.26101758508583744, -0.36918909004000194, -0.4774057077440092, -0.5855741545088923, -0.6936007617056474, -0.8013921282596846, -0.9088554475010583, -1.0158987488307232, -1.1224311301764076, -1.2283629804081164, -1.3336061900901708, -1.4380743489707304, -1.5416829287449212, -1.644349449799583, -1.745993630793385, -1.846537520109921, -1.945905608444004, -2.0440249218939375, -2.140825095132479, -2.236238424531952, -2.330199901163478, -2.4226472237461567, -2.513520791918424, -2.602763680267308, -2.6903215936335894, -2.776142804408091, -2.8601780726784805, -2.942380550091265, -3.0227056683830273, -3.1011110136037767, -3.1775561870950875, -3.2520026543012386, -3.324413582476476, -3.3947536683852344, -3.4629889570547485, -3.529086652581191, -3.5930149220124163, -3.6547426932013205, -3.7142394476204807, -3.771475008796764, -3.826419327257709, -3.879042262924192, -3.929313352612598, -3.9772014916863028, -4.022674902172897, -4.065700925152941, -4.106245613162739, -4.144273585678045, -4.179747804578863, -4.212629361727562, -4.242877270892289, -4.270448224975226, -4.295296368136567, -4.317373097951761, -4.336626856750177, -4.353002928578305, -4.366443256437268, -4.37688624622736, -4.384266561201013, -4.3885149623944875, -4.389558167722866, -4.38731871501979, -4.3817149009498655, -4.372660610236323, -4.360065179848289, -4.343833507435143, -4.323866089496796, -4.300059036658276, -4.272304197477523, -4.240489348632538, -4.204498461288362, -4.164212054441277, -4.119507647058946, -4.070260321668487, -4.0134701736599006, -3.9462603304997774;

        calf        << -1.8176489481619973, -1.8175203888819933, -1.8170833289770902, -1.8163372887792215, -1.8152820794250673, -1.8139177784978593, -1.8122443826775856, -1.8102619930349606, -1.8079708590724362, -1.805371379958805, -1.8024641052048642, -1.7992497352782744, -1.795729122215822, -1.7919032702340671, -1.7877733363385013, -1.7833406309316115, -1.7786066184205662, -1.7735729178257054, -1.7682413033914102, -1.7626137052015858, -1.7566922098024482, -1.7504790608359455, -1.7439766596879769, -1.737187566156201, -1.7301144991427215, -1.7227603373781304, -1.7151281201839952, -1.7072210482816117, -1.6990424846557886, -1.6905959554834094, -1.6818851511371546, -1.6729139272755789, -1.6636863060316573, -1.6542064773126846, -1.6444788002251987, -1.6345078046393045, -1.6242981929075404, -1.6138548417541203, -1.6031828043509186, -1.5922873125972101, -1.5811737796206844, -1.5698478025175642, -1.5583151653502132, -1.546581842420333, -1.5346540018371124, -1.5225380093979555, -1.510240432809754, -1.4977680463816185, -1.4851278362709928, -1.4723270051061303, -1.459372977952172, -1.446273408527825, -1.4330361857956093, -1.419669440991047, -1.406181555055682, -1.3925811665324839, -1.3788771800366835, -1.365078775138976, -1.3511954156931454, -1.3372368596075133, -1.323213169028298, -1.309134720870035, -1.2950122178012506, -1.2808566995728776, -1.2666795545388927, -1.2524925313468325, -1.2383077506705555, -1.2241377167609195, -1.2099953294155672, -1.1958938953788996, -1.1818471388844187, -1.1678692116835887, -1.15397470216529, -1.140178643244468, -1.1264965187054554, -1.1129442676471946, -1.0995382866378112, -1.0862954291447566, -1.0732354050883937, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2129405125734207, -1.2269710629359707, -1.240182170245854, -1.2532881052365765, -1.2664290260088924, -1.2795960854904584, -1.2927787833002922, -1.3059666747493606, -1.3191495538856424, -1.3323174515546972, -1.3454606300092848, -1.3585695772181563, -1.371635001173093, -1.3846478242497653, -1.3975991776236971, -1.410480395623932, -1.4232830105797039, -1.4359987481052292, -1.4486195220436628, -1.461137429766426, -1.4735447477393522, -1.485833927331081, -1.4979975909337873, -1.5100285282558168, -1.5219196928039962, -1.5336641986468633, -1.5452553174238974, -1.556686475587162, -1.5679512518831875, -1.5790433750989916, -1.5899567218990376, -1.6006853148836777, -1.6112233208797109, -1.6215650494141176, -1.6317049514154907, -1.641637618239087, -1.651357779981943, -1.6608603048385162, -1.6701401984354591, -1.679192603178251, -1.6880127976983679, -1.6965961964000935, -1.7049383490967052, -1.7130349407267151, -1.7208817911408447, -1.7284748549509705, -1.735810221432908, -1.7428841144753455, -1.7496928925678468, -1.7562330488213251, -1.7625012110152152, -1.7684941416660056, -1.7742087381125176, -1.7796420326137425, -1.7847911924560205, -1.7896535200666004, -1.7942264531314127, -1.7985075647153521, -1.8024945633840257, -1.8061852933263023, -1.809577734477509, -1.8126700026435678, -1.8154603496264703, -1.8179471633519806, -1.8201289680007973, -1.8220044241444215, -1.8235723288871513, -1.8248316160157247, -1.8257813561582066, -1.826420756953555, -1.8267491632333814, -1.8267660572172018;
        calf_vel    << -0.012728582818290257, 0.14136572645037992, 0.29574658096409245, 0.45030044706315364, 0.6048666577591358, 0.7594083472247475, 0.9139361161720638, 1.0683763580505587, 1.2226544805209085, 1.3766954703376058, 1.530424005435881, 1.6837644572463233, 1.8366408915984727, 1.9889770695093634, 2.1406964478631245, 2.291722179818512, 2.441977114743076, 2.591383797465508, 2.73986446657874, 2.8873410515087308, 3.033735168093283, 3.178968112288632, 3.3229608516189795, 3.4656340141313904, 3.6069078744134586, 3.7467023361963174, 3.884936911246486, 4.02153069412105, 4.156402332280645, 4.289469991162436, 4.4206513138735675, 4.54986337505136, 4.677022628470297, 4.80204484800736, 4.9248450615936346, 5.045337477787515, 5.163435404592173, 5.279051160222512, 5.3920959755271625, 5.502479887760344, 5.610111625524912, 5.714898484626659, 5.816746194849134, 5.915558777299635, 6.01123839247436, 6.1036851794172415, 6.192797066354123, 6.27846944379968, 6.360595268932096, 6.439064893064788, 6.513765571650152, 6.584581375306751, 6.651392968596235, 6.714077400646962, 6.772507896115406, 6.826553585914277, 6.87607925675998, 6.920945144875837, 6.961006710282506, 6.996114419526798, 7.026113562093225, 7.050844048747245, 7.070140200129952, 7.083830617781749, 7.091738095966865, 7.093679553203764, 7.089466103765033, 7.078902969182236, 7.061789470456181, 7.03791944880329, 7.007081598263906, 6.969059792502778, 6.923633627523444, 6.8705791124814635, 6.809669527155686, 6.740676465625836, 6.663371086499846, 6.577525590246305, 6.478104071356169, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -7.057857664906111, -6.812157426482067, -6.543436764366062, -6.562096840962303, -6.577607786969333, -6.588073973294019, -6.593260314262045, -6.593284084884066, -6.588264734599222, -6.5783193763363945, -6.563562291878566, -6.5441049111157525, -6.520055826717827, -6.491520832526897, -6.4586028979888255, -6.4214022676016365, -6.380016757242389, -6.334541739913211, -6.285070122410402, -6.231692537364905, -6.1744974567848585, -6.113571337594816, -6.048998745358483, -5.980862434126296, -5.90924346752657, -5.834221354606815, -5.755874167469017, -5.674278656754954, -5.58951038664074, -5.501643818842247, -5.410752382600457, -5.316908596948526, -5.220184174338066, -5.1206501070908494, -5.018376866781672, -4.9134342265455615, -4.805891347112786, -4.695817111797364, -4.583280061416934, -4.468348433919409, -4.351090216627983, -4.2315731934849685, -4.109864987371836, -3.9860330977929825, -3.8601449340059353, -3.732267843936759, -3.6024691391381665, -3.4708161160547952, -3.3373760738583367, -3.2022163291922485, -3.065404228141532, -2.927007155715663, -2.787092543118856, -2.6457278731760376, -2.5029806842223143, -2.3589185726992326, -2.213609194789256, -2.0671202673656572, -1.9195195685214343, -1.7708749378872997, -1.6212542770028344, -1.4707255498575695, -1.319356783741936, -1.167216070649688, -1.0143715693044413, -0.8608915078484444, -0.7068441872712324, -0.5522979856278056, -0.39732136301291543, -0.24198286727493923, -0.0863511404398335, 0.06950507423924743;
        calf_f      << -1.8193203295354938, -1.8190451301704618, -1.818469254589085, -1.817592215310493, -1.8164134313302278, -1.8149324129010418, -1.8131488063572747, -1.811062396178327, -1.8086731064993695, -1.8059810025621748, -1.802986292162075, -1.7996893270897998, -1.7960906045660348, -1.7921907686667515, -1.7879906117376336, -1.7834910757963438, -1.7786932539217517, -1.7735983916298743, -1.7682078882367482, -1.7625232982090995, -1.7565463325045434, -1.750278859903729, -1.7437229083373949, -1.736880666212588, -1.7297544837430028, -1.7223468742892551, -1.714660515715985, -1.7066982517738298, -1.6984630935151084, -1.6899582207530692, -1.6811869835756774, -1.6721529039259038, -1.6628596772615143, -1.6533111743083047, -1.6435114429217677, -1.6334647100731519, -1.6231753839767005, -1.612648056375829, -1.6018875050068329, -1.5908986962593912, -1.579686788054061, -1.5682571329570731, -1.5566152815544525, -1.5447669861062074, -1.5327182045123444, -1.5204751047281035, -1.5080440697194633, -1.4954317017710301, -1.4826448281273092, -1.469690506887454, -1.4565760332818625, -1.4433089464044397, -1.4298970363748467, -1.4163483520004096, -1.4026712090655815, -1.3888741991010005, -1.3749661986798531, -1.3609563792591697, -1.3468542175544655, -1.3326695064048335, -1.3184123662627993, -1.3040932572261812, -1.2897229914919681, -1.2753127462430418, -1.2608740768766478, -1.2464189303879938, -1.2319596595595406, -1.2175090370184956, -1.2030802689156486, -1.1886870086191308, -1.174343370084397, -1.1600639406343494, -1.1458637928895206, -1.1317584955482127, -1.1177641226730226, -1.103897261092871, -1.0901774480876125, -1.0766394251401175;
        calf_f_vel  << 0.06255710680916078, 0.2127246334332622, 0.36318522497076294, 0.5139067208650699, 0.664906455921324, 0.816117200713706, 0.9674703313461515, 1.1188963861365901, 1.2703251766833201, 1.4216857902667561, 1.5729065919442362, 1.7239152276570018, 1.874638628386393, 2.0250030152231555, 2.1749339051708474, 2.3243561174921283, 2.4731937803411386, 2.6213703374006037, 2.7688085542654375, 2.9154305241786598, 3.0611576727134664, 3.2059107611369337, 3.349609887975584, 3.4921744882568846, 3.6335233300750875, 3.773574507991681, 3.912245432689612, 4.049452816399769, 4.185112653669882, 4.3191401969229535, 4.45144992627765, 4.5819555131270135, 4.710569776980883, 4.8372046350773665, 4.961771044247848, 5.084178934595196, 5.204337134537189, 5.322153286746768, 5.437533754638411, 5.55038351895665, 5.6606060642822955, 5.768103254895051, 5.8727751999141065, 5.974520107853407, 6.073234110352758, 6.168810943320158, 6.261142060800279, 6.350116471973544, 6.435620252600701, 6.517536453106288, 6.59574487132291, 6.670121832471997, 6.74053996449238, 6.806867906588771, 6.868970028982684, 6.926706189231165, 6.979931460729779, 7.028495859957666, 7.072244096919891, 7.1110152955984445, 7.144642690551159, 7.172953390823828, 7.195768168877413, 7.21290125166174, 7.2241602334234, 7.229345806924642, 7.228251549158192, 7.2206641127553555, 7.206363307482541, 7.185122148723407, 7.1567070934246, 7.120878396742785, 7.077390608427095, 7.025993229810424, 6.96643155415391, 6.8984477146255845, 6.816899165286185, 6.716798186241844;

        time_f      << 0.0, 0.0020000000000000018, 0.0040000000000000036, 0.006000000000000005, 0.008000000000000007, 0.010000000000000009, 0.01200000000000001, 0.014000000000000012, 0.015999999999999986, 0.017999999999999988, 0.01999999999999999, 0.021999999999999992, 0.023999999999999994, 0.025999999999999995, 0.027999999999999997, 0.03, 0.032, 0.034, 0.036000000000000004, 0.038000000000000006, 0.04000000000000001, 0.04200000000000001, 0.04400000000000001, 0.04600000000000001, 0.048000000000000015, 0.05000000000000002, 0.05200000000000002, 0.05400000000000002, 0.05600000000000002, 0.058000000000000024, 0.060000000000000026, 0.06200000000000003, 0.06400000000000003, 0.06600000000000003, 0.06800000000000003, 0.07000000000000003, 0.07200000000000004, 0.07400000000000004, 0.07600000000000004, 0.07800000000000004, 0.08000000000000004, 0.08200000000000005, 0.08400000000000005, 0.08600000000000005, 0.08800000000000005, 0.09000000000000005, 0.09200000000000005, 0.09400000000000006, 0.09600000000000006, 0.09800000000000006, 0.10000000000000006, 0.10200000000000006, 0.10400000000000006, 0.10600000000000007, 0.10800000000000007, 0.11000000000000007, 0.11200000000000007, 0.11400000000000007, 0.11600000000000008, 0.11800000000000008, 0.12000000000000008, 0.12200000000000008, 0.12400000000000008, 0.12600000000000008, 0.12800000000000009, 0.1300000000000001, 0.1320000000000001, 0.1340000000000001, 0.1360000000000001, 0.1380000000000001, 0.1400000000000001, 0.1420000000000001, 0.1440000000000001, 0.1460000000000001, 0.1480000000000001, 0.1500000000000001, 0.1520000000000001, 0.15399999952316293;


        
        // ifstream fin("DATA.yaml"); 
        // Node config = Load(fin);

        // // Read data from the YAML file
        // hip << config["hip"].as<double>();
        // hip_vel << config["hip_vel"].as<double>();
        // hip_f << config["hip_f"].as<double>();
        // hip_f_vel << config["hip_f_vel"].as<double>();

        // thigh << config["thigh"].as<double>();
        // thigh_vel << config["thigh_vel"].as<double>();
        // thigh_f << config["thigh_f"].as<double>();
        // thigh_f_vel << config["thigh_f_vel"].as<double>();

        // calf << config["calf"].as<double>();
        // calf_vel << config["calf_vel"].as<double>();
        // calf_f << config["calf_f"].as<double>();
        // calf_f_vel << config["calf_f_vel"].as<double>();

        // time_f << config["time_f"].as<double>();
        
}

Vec316<double> Data::get_pose_data(int link){
        switch (link) {
        case 0:
                return hip;
                break;
        case 1:
                return thigh;
                break;
        case 2:
                return calf;
                break;
        }
}

Vec316<double> Data::get_vel_data(int link){
        switch (link) {
        case 0:
                return hip_vel;
                break;
        case 1:
                return thigh_vel;
                break;
        case 2:
                return calf_vel;
                break;
        }
}

Vec78<double> Data::get_pose_data_f(int link){
        switch (link) {
        case 0:
                return hip_f;
                break;
        case 1:
                return thigh_f;
                break;
        case 2:
                return calf_f;
                break;
        }
}
Vec78<double> Data::get_vel_data_f(int link){
        switch (link) {
        case 0:
                return hip_f_vel;
                break;
        case 1:
                return thigh_f_vel;
                break;
        case 2:
                return calf_f_vel;
                break;
        }
}

