window.mwPerformance=(window.performance&&performance.mark)?performance:{mark:function(){}};window.mwNow=(function(){var perf=window.performance,navStart=perf&&perf.timing&&perf.timing.navigationStart;return navStart&&typeof perf.now==='function'?function(){return navStart+perf.now();}:function(){return Date.now();};}());window.isCompatible=function(str){var ua=str||navigator.userAgent;return!!((function(){'use strict';return!this&&!!Function.prototype.bind&&!!window.JSON;}())&&'querySelector'in document&&'localStorage'in window&&'addEventListener'in window&&!(ua.match(/MSIE 10|webOS\/1\.[0-4]|SymbianOS|Series60|NetFront|Opera Mini|S40OviBrowser|MeeGo|Android.+Glass|^Mozilla\/5\.0 .+ Gecko\/$|googleweblight/)||ua.match(/PlayStation/i)));};(function(){var NORLQ,script;if(!isCompatible()){document.documentElement.className=document.documentElement.className.replace(/(^|\s)client-js(\s|$)/,'$1client-nojs$2');NORLQ=window.NORLQ||[];while(NORLQ.length){NORLQ.shift()();}window.NORLQ={push:
function(fn){fn();}};window.RLQ={push:function(){}};return;}function startUp(){mw.config=new mw.Map(true);mw.loader.addSource({"local":"/load.php"});mw.loader.register([["site","0384gx0",[1]],["site.styles","1dt3njg",[],"site"],["noscript","0mep275",[],"noscript"],["filepage","1nd8x1u"],["user.groups","0i0c44c",[5]],["user","08wpci8",[6],"user"],["user.styles","1al5ooy",[],"user"],["user.defaults","1hatv5n"],["user.options","0r5ungb",[7],"private"],["user.tokens","0qn2kg0",[],"private"],["mediawiki.language.data","1pox0t6",[177]],["mediawiki.skinning.elements","18kbr3o"],["mediawiki.skinning.content","0vfbc53"],["mediawiki.skinning.interface","0wvsj33"],["mediawiki.skinning.content.parsoid","0zm9icd"],["mediawiki.skinning.content.externallinks","0eak6fs"],["jquery.accessKeyLabel","1q08xze",[22,130]],["jquery.async","0cohsnf"],["jquery.byteLength","189euku",[131]],["jquery.byteLimit","0i0c44c",[37]],["jquery.checkboxShiftClick","1vvld4d"],["jquery.chosen","1tckqb4"],["jquery.client",
"0ka1sv4"],["jquery.color","1vtudft",[24]],["jquery.colorUtil","03gzfwa"],["jquery.confirmable","1xxkah9",[178]],["jquery.cookie","19g0e10"],["jquery.expandableField","0wwt5a6"],["jquery.farbtastic","1fr9oed",[24]],["jquery.footHovzer","1cffzho"],["jquery.form","1sexd2p"],["jquery.fullscreen","0fhl1px"],["jquery.getAttrs","1ugle02"],["jquery.hidpi","0o05vhi"],["jquery.highlightText","18pfe9e",[130]],["jquery.hoverIntent","0g0me8v"],["jquery.i18n","0m49ab2",[176]],["jquery.lengthLimit","0raoy7q",[131]],["jquery.localize","1e2c73f"],["jquery.makeCollapsible","0g70azt"],["jquery.mockjax","0ioay3c"],["jquery.mw-jump","100gx0a"],["jquery.qunit","01vadlw"],["jquery.spinner","100xupq"],["jquery.jStorage","0yfali9"],["jquery.suggestions","1tv0vh2",[34]],["jquery.tabIndex","1iz0e0j"],["jquery.tablesorter","1q74kbe",[130,179]],["jquery.textSelection","0f2wes3",[22]],["jquery.throttle-debounce","1ea3xiw"],["jquery.xmldom","1jc871f"],["jquery.tipsy","07b5tm1"],["jquery.ui.core","0a7p455",[53],
"jquery.ui"],["jquery.ui.core.styles","120yvbf",[],"jquery.ui"],["jquery.ui.accordion","1s1l3iw",[52,72],"jquery.ui"],["jquery.ui.autocomplete","1m22tld",[61],"jquery.ui"],["jquery.ui.button","0t4xb94",[52,72],"jquery.ui"],["jquery.ui.datepicker","0rv8mwi",[52],"jquery.ui"],["jquery.ui.dialog","0xjrjeg",[56,59,63,65],"jquery.ui"],["jquery.ui.draggable","0ljld7m",[52,62],"jquery.ui"],["jquery.ui.droppable","1cz190r",[59],"jquery.ui"],["jquery.ui.menu","0fin5fy",[52,63,72],"jquery.ui"],["jquery.ui.mouse","0pee7b2",[72],"jquery.ui"],["jquery.ui.position","03e3dpj",[],"jquery.ui"],["jquery.ui.progressbar","04kzfrs",[52,72],"jquery.ui"],["jquery.ui.resizable","0p4re3l",[52,62],"jquery.ui"],["jquery.ui.selectable","0hnc9e6",[52,62],"jquery.ui"],["jquery.ui.slider","0lzhfea",[52,62],"jquery.ui"],["jquery.ui.sortable","1xtu6em",[52,62],"jquery.ui"],["jquery.ui.spinner","0egx87q",[56],"jquery.ui"],["jquery.ui.tabs","1cyz4yt",[52,72],"jquery.ui"],["jquery.ui.tooltip","1tl06bp",[52,63,72],
"jquery.ui"],["jquery.ui.widget","0s4c95g",[],"jquery.ui"],["jquery.effects.core","11y9zjl",[],"jquery.ui"],["jquery.effects.blind","0t686g0",[73],"jquery.ui"],["jquery.effects.bounce","1hx0nxw",[73],"jquery.ui"],["jquery.effects.clip","1hpfd5x",[73],"jquery.ui"],["jquery.effects.drop","0fk0ebd",[73],"jquery.ui"],["jquery.effects.explode","1llz6nu",[73],"jquery.ui"],["jquery.effects.fade","0p09gts",[73],"jquery.ui"],["jquery.effects.fold","1p1koj8",[73],"jquery.ui"],["jquery.effects.highlight","1656o3a",[73],"jquery.ui"],["jquery.effects.pulsate","0zcrlqu",[73],"jquery.ui"],["jquery.effects.scale","1aazboy",[73],"jquery.ui"],["jquery.effects.shake","07yuo3l",[73],"jquery.ui"],["jquery.effects.slide","1w6qlam",[73],"jquery.ui"],["jquery.effects.transfer","0jnya70",[73],"jquery.ui"],["json","0i0c44c"],["moment","0sd2icq",[174]],["mediawiki.apihelp","198xa0h"],["mediawiki.template","0bc0g01"],["mediawiki.template.mustache","12fsbol",[90]],["mediawiki.template.regexp","1oppoav",[90]],[
"mediawiki.apipretty","0bsntbz"],["mediawiki.api","1w6qlvc",[148,9]],["mediawiki.api.category","0qpsla0",[136,94]],["mediawiki.api.edit","0ljb5b7",[146]],["mediawiki.api.login","0jf47gk",[94]],["mediawiki.api.options","0urefca",[94]],["mediawiki.api.parse","1uo70l0",[94]],["mediawiki.api.upload","0xv2foy",[96]],["mediawiki.api.user","1yrdg76",[94]],["mediawiki.api.watch","0kmhhoe",[94]],["mediawiki.api.messages","1vfo2qn",[94]],["mediawiki.api.rollback","1t22euw",[94]],["mediawiki.content.json","0ldzwkr"],["mediawiki.confirmCloseWindow","1q0u7bd"],["mediawiki.debug","0lbt5s3",[29,273]],["mediawiki.diff.styles","1n4gq06"],["mediawiki.feedback","1lmjiyz",[136,124,277]],["mediawiki.feedlink","06tol7p"],["mediawiki.filewarning","0h1cleo",[273]],["mediawiki.ForeignApi","11nmc76",[113]],["mediawiki.ForeignApi.core","0h7dgi2",[94,269]],["mediawiki.helplink","03fto6l"],["mediawiki.hidpi","16bq0hc",[33],null,null,"return'srcset'in new Image();"],["mediawiki.hlist","1b97zjv"],[
"mediawiki.htmlform","0zrpxet",[37,130]],["mediawiki.htmlform.checker","0eyc9g7",[49]],["mediawiki.htmlform.ooui","1pmg65l",[273]],["mediawiki.htmlform.styles","054d0ip"],["mediawiki.htmlform.ooui.styles","0nu18sy"],["mediawiki.icon","0yr8ehz"],["mediawiki.inspect","0vlra09",[130,131]],["mediawiki.messagePoster","0tyqcoz",[112]],["mediawiki.messagePoster.wikitext","1eeuzng",[96,124]],["mediawiki.notification","0e409xs",[148,156]],["mediawiki.notify","0018xm0"],["mediawiki.notification.convertmessagebox","06ce8rp",[126]],["mediawiki.notification.convertmessagebox.styles","1vd2u4m"],["mediawiki.RegExp","1mkq31w"],["mediawiki.String","17i214i"],["mediawiki.pager.tablePager","188vdyh"],["mediawiki.searchSuggest","1gp0zy4",[32,45,94]],["mediawiki.sectionAnchor","18uy4jl"],["mediawiki.storage","0rskhx7"],["mediawiki.Title","02eazfp",[131,148]],["mediawiki.Upload","0yegwa8",[100]],["mediawiki.ForeignUpload","138cpsa",[112,137]],["mediawiki.ForeignStructuredUpload.config","0msydx9"],[
"mediawiki.ForeignStructuredUpload","1cvqbgr",[139,138]],["mediawiki.Upload.Dialog","02xtom1",[142]],["mediawiki.Upload.BookletLayout","1y4lio4",[136,137,178,266,88,275,277]],["mediawiki.ForeignStructuredUpload.BookletLayout","0rs6ge6",[140,142,103,182,256,251]],["mediawiki.toc","1ttetmy",[152]],["mediawiki.Uri","0rxgn9c",[148,92]],["mediawiki.user","0o9pm8e",[101,135,8]],["mediawiki.userSuggest","187j7c7",[45,94]],["mediawiki.util","1hwa75w",[16,127]],["mediawiki.viewport","1lypsjv"],["mediawiki.checkboxtoggle","15s4gdc"],["mediawiki.checkboxtoggle.styles","1yq8yat"],["mediawiki.cookie","1esdtvi",[26]],["mediawiki.toolbar","1mjx67k",[48]],["mediawiki.experiments","1sglc2n"],["mediawiki.editfont.styles","0lfsqtx"],["mediawiki.visibleTimeout","0egypxx"],["mediawiki.action.delete","1sslgfv",[37,273]],["mediawiki.action.delete.file","090erxs",[37]],["mediawiki.action.edit","16lrx59",[48,160,94,155,254]],["mediawiki.action.edit.styles","03p2yr1"],["mediawiki.action.edit.collapsibleFooter",
"11u61dv",[39,122,135]],["mediawiki.action.edit.preview","1d2f6cw",[43,48,94,108,178,273]],["mediawiki.action.history","07ktec0"],["mediawiki.action.history.styles","02lxzow"],["mediawiki.action.view.dblClickEdit","07i78c1",[148,8]],["mediawiki.action.view.metadata","1cz503s",[173]],["mediawiki.action.view.categoryPage.styles","0d4tnuf"],["mediawiki.action.view.postEdit","1j3gh3m",[178,126]],["mediawiki.action.view.redirect","1s44gpl",[22]],["mediawiki.action.view.redirectPage","1q892z9"],["mediawiki.action.view.rightClickEdit","160gtnj"],["mediawiki.action.edit.editWarning","1rfkg5i",[48,106,178]],["mediawiki.action.view.filepage","0ow4z76"],["mediawiki.language","0xrl4wr",[175,10]],["mediawiki.cldr","0xlguph",[176]],["mediawiki.libs.pluralruleparser","174vfen"],["mediawiki.language.init","17w370b"],["mediawiki.jqueryMsg","1hfds3n",[174,148,8]],["mediawiki.language.months","039qsii",[174]],["mediawiki.language.names","0cfo43y",[177]],["mediawiki.language.specialCharacters","0meovla",[
174]],["mediawiki.libs.jpegmeta","1wvtpje"],["mediawiki.page.gallery","0z4eh1x",[49,184]],["mediawiki.page.gallery.styles","16n1vo2"],["mediawiki.page.gallery.slideshow","0mob4cl",[136,94,275,290]],["mediawiki.page.ready","1t2gn7c",[16,20,41]],["mediawiki.page.startup","1n2mycb"],["mediawiki.page.patrol.ajax","0epxiwh",[43,136,94]],["mediawiki.page.watch.ajax","10poqjj",[136,102,178]],["mediawiki.page.rollback","0rl4gq7",[43,104]],["mediawiki.page.image.pagination","1gmhh2q",[43,148]],["mediawiki.rcfilters.filters.base.styles","1uf6rbw"],["mediawiki.rcfilters.highlightCircles.seenunseen.styles","19udd90"],["mediawiki.rcfilters.filters.dm","0fegxpu",[131,145,98,178,146,269]],["mediawiki.rcfilters.filters.ui","1w94tyd",[39,194,249,284,286,288,290]],["mediawiki.special","0xkp65j"],["mediawiki.special.apisandbox.styles","025eisv"],["mediawiki.special.apisandbox","0ea2d1h",[39,94,178,255,272]],["mediawiki.special.block","0li3s9v",[117,148,257]],["mediawiki.special.changecredentials.js",
"12q889c",[94,119]],["mediawiki.special.changeslist","0e59dg3"],["mediawiki.special.changeslist.enhanced","1btyh8b"],["mediawiki.special.changeslist.legend","0uidzxr"],["mediawiki.special.changeslist.legend.js","1uvoiwm",[39,152]],["mediawiki.special.changeslist.visitedstatus","00oesjr"],["mediawiki.special.comparepages.styles","0m361zi"],["mediawiki.special.contributions","07ni0lx",[178,251]],["mediawiki.special.edittags","0nogs0c",[21,37]],["mediawiki.special.edittags.styles","0iwkr05"],["mediawiki.special.import","0en5oo4"],["mediawiki.special.movePage","0itov8d",[249,254]],["mediawiki.special.movePage.styles","0ypd97d"],["mediawiki.special.pageLanguage","1lbm7v9",[273]],["mediawiki.special.pagesWithProp","04c1p0m"],["mediawiki.special.preferences","1hapb6m",[106,174,128]],["mediawiki.special.preferences.styles","0owlvfw"],["mediawiki.special.recentchanges","1g3nt96"],["mediawiki.special.revisionDelete","05mq55m",[37]],["mediawiki.special.search","0j1yzjy",[264]],[
"mediawiki.special.search.commonsInterwikiWidget","0mit4qj",[145,94,178]],["mediawiki.special.search.interwikiwidget.styles","0wznblo"],["mediawiki.special.search.styles","0f0pf0x"],["mediawiki.special.undelete","1gheg34",[249,254]],["mediawiki.special.unwatchedPages","0z7dbda",[136,102]],["mediawiki.special.upload","0mo934d",[43,136,99,106,178,182,226,90]],["mediawiki.special.upload.styles","03jk9mt"],["mediawiki.special.userlogin.common.styles","15bsc5y"],["mediawiki.special.userlogin.login.styles","0clbl8u"],["mediawiki.special.userlogin.signup.js","0oh5433",[94,118,178]],["mediawiki.special.userlogin.signup.styles","0oqetts"],["mediawiki.special.userrights","0uieh0r",[37,128]],["mediawiki.special.watchlist","134op1v",[136,102,178,273]],["mediawiki.special.watchlist.styles","0qr67oy"],["mediawiki.special.version","0rwb3m4"],["mediawiki.legacy.config","1wd9qhc"],["mediawiki.legacy.commonPrint","0xe9mk5"],["mediawiki.legacy.protect","120zcr8",[37]],["mediawiki.legacy.shared","1urji1d"
],["mediawiki.legacy.oldshared","0vb340f"],["mediawiki.legacy.wikibits","0634ays"],["mediawiki.ui","1ijoc1d"],["mediawiki.ui.checkbox","18wab5y"],["mediawiki.ui.radio","0sta8so"],["mediawiki.ui.anchor","0u9dv43"],["mediawiki.ui.button","0zqcbqo"],["mediawiki.ui.input","1rbjpoy"],["mediawiki.ui.icon","0ubmabt"],["mediawiki.ui.text","0adobvo"],["mediawiki.widgets","0044eqk",[136,94,250,275]],["mediawiki.widgets.styles","0g0yn1r"],["mediawiki.widgets.DateInputWidget","1x3ok08",[252,88,275]],["mediawiki.widgets.DateInputWidget.styles","0ag7v6z"],["mediawiki.widgets.visibleByteLimit","0i0c44c",[254]],["mediawiki.widgets.visibleLengthLimit","1cr3rq1",[37,273]],["mediawiki.widgets.datetime","0d7hvig",[273,291,292]],["mediawiki.widgets.CategoryMultiselectWidget","1mmljvc",[112,136,275]],["mediawiki.widgets.SelectWithInputWidget","0d21ygm",[258,275]],["mediawiki.widgets.SelectWithInputWidget.styles","0rnsevd"],["mediawiki.widgets.SizeFilterWidget","1axfs25",[260,275]],[
"mediawiki.widgets.SizeFilterWidget.styles","18jd9l4"],["mediawiki.widgets.MediaSearch","0g8cebg",[112,136,275]],["mediawiki.widgets.UserInputWidget","1us59dk",[94,275]],["mediawiki.widgets.UsersMultiselectWidget","0yaasr6",[94,275]],["mediawiki.widgets.SearchInputWidget","14khb2h",[133,249]],["mediawiki.widgets.SearchInputWidget.styles","08nibyd"],["mediawiki.widgets.StashedFileWidget","0l5gkeb",[94,273]],["es5-shim","0i0c44c"],["dom-level2-shim","0i0c44c"],["oojs","0jl7ngg"],["mediawiki.router","0ohx2zp",[271]],["oojs-router","0j1t1g5",[269]],["oojs-ui","0i0c44c",[276,275,277]],["oojs-ui-core","0tcidn6",[174,269,274,281,282,287,278,279]],["oojs-ui-core.styles","111dla1"],["oojs-ui-widgets","1neii5h",[273,283,291,292]],["oojs-ui-toolbars","0fjnfmc",[273,292]],["oojs-ui-windows","1kcdw7d",[273,292]],["oojs-ui.styles.indicators","05ylh5f"],["oojs-ui.styles.textures","1spy1fd"],["oojs-ui.styles.icons-accessibility","1vj9vbq"],["oojs-ui.styles.icons-alerts","0fk0ori"],[
"oojs-ui.styles.icons-content","11eopdq"],["oojs-ui.styles.icons-editing-advanced","1h2f081"],["oojs-ui.styles.icons-editing-core","1wxzoac"],["oojs-ui.styles.icons-editing-list","1828hac"],["oojs-ui.styles.icons-editing-styling","1w46831"],["oojs-ui.styles.icons-interactions","0t5k9hh"],["oojs-ui.styles.icons-layout","05geezy"],["oojs-ui.styles.icons-location","1w7xw0a"],["oojs-ui.styles.icons-media","0sv4o9d"],["oojs-ui.styles.icons-moderation","16qxjwr"],["oojs-ui.styles.icons-movement","14hmdfw"],["oojs-ui.styles.icons-user","18q23fg"],["oojs-ui.styles.icons-wikimedia","0s3ae1q"],["skins.cologneblue","1hstfa7"],["skins.modern","11gr99v"],["skins.monobook.styles","002m6cw"],["skins.vector.styles","0jh4skm"],["skins.vector.styles.responsive","06s45tv"],["skins.vector.js","04u4k5c",[46,49]],["ext.wikiEditor","1jhm4yo",[17,26,45,46,48,58,143,141,181,284,285,286,290,90],"ext.wikiEditor"],["ext.wikiEditor.styles","0fhgtxp",[],"ext.wikiEditor"],["ext.wikiEditor.toolbar","0i0c44c",[301]],[
"ext.wikiEditor.dialogs","0i0c44c",[301]],["ext.wikiEditor.core","0i0c44c",[301]],["jquery.wikiEditor","0i0c44c",[301]],["jquery.wikiEditor.core","0i0c44c",[301]],["jquery.wikiEditor.dialogs","0i0c44c",[301]],["jquery.wikiEditor.dialogs.config","0i0c44c",[301]],["jquery.wikiEditor.toolbar","0i0c44c",[301]],["jquery.wikiEditor.toolbar.config","0i0c44c",[301]],["jquery.wikiEditor.toolbar.i18n","0i0c44c",[301]]]);;mw.config.set({"wgLoadScript":"/load.php","debug":!1,"skin":"vector","stylepath":"/skins","wgUrlProtocols":"bitcoin\\:|ftp\\:\\/\\/|ftps\\:\\/\\/|geo\\:|git\\:\\/\\/|gopher\\:\\/\\/|http\\:\\/\\/|https\\:\\/\\/|irc\\:\\/\\/|ircs\\:\\/\\/|magnet\\:|mailto\\:|mms\\:\\/\\/|news\\:|nntp\\:\\/\\/|redis\\:\\/\\/|sftp\\:\\/\\/|sip\\:|sips\\:|sms\\:|ssh\\:\\/\\/|svn\\:\\/\\/|tel\\:|telnet\\:\\/\\/|urn\\:|worldwind\\:\\/\\/|xmpp\\:|\\/\\/","wgArticlePath":"/$1","wgScriptPath":"","wgScript":"/index.php","wgSearchType":null,"wgVariantArticlePath":!1,"wgActionPaths":{},"wgServer":
"//wiki.keyestudio.com","wgServerName":"wiki.keyestudio.com","wgUserLanguage":"en","wgContentLanguage":"en","wgTranslateNumerals":!0,"wgVersion":"1.31.1","wgEnableAPI":!0,"wgEnableWriteAPI":!0,"wgMainPageTitle":"Main Page","wgFormattedNamespaces":{"-2":"Media","-1":"Special","0":"","1":"Talk","2":"User","3":"User talk","4":"Keyestudio Wiki","5":"Keyestudio Wiki talk","6":"File","7":"File talk","8":"MediaWiki","9":"MediaWiki talk","10":"Template","11":"Template talk","12":"Help","13":"Help talk","14":"Category","15":"Category talk"},"wgNamespaceIds":{"media":-2,"special":-1,"":0,"talk":1,"user":2,"user_talk":3,"keyestudio_wiki":4,"keyestudio_wiki_talk":5,"file":6,"file_talk":7,"mediawiki":8,"mediawiki_talk":9,"template":10,"template_talk":11,"help":12,"help_talk":13,"category":14,"category_talk":15,"image":6,"image_talk":7,"project":4,"project_talk":5},"wgContentNamespaces":[0],"wgSiteName":"Keyestudio Wiki","wgDBname":"my_wiki","wgExtraSignatureNamespaces":[],"wgAvailableSkins":{
"cologneblue":"CologneBlue","modern":"Modern","monobook":"MonoBook","vector":"Vector","fallback":"Fallback","apioutput":"ApiOutput"},"wgExtensionAssetsPath":"/extensions","wgCookiePrefix":"my_wiki","wgCookieDomain":"","wgCookiePath":"/","wgCookieExpiration":2592000,"wgResourceLoaderMaxQueryLength":2000,"wgCaseSensitiveNamespaces":[],"wgLegalTitleChars":" %!\"$&'()*,\\-./0-9:;=?@A-Z\\\\\\^_`a-z~+\\u0080-\\uFFFF","wgIllegalFileChars":":/\\\\","wgResourceLoaderStorageVersion":1,"wgResourceLoaderStorageEnabled":!0,"wgForeignUploadTargets":["local"],"wgEnableUploads":!0,"wgCommentByteLimit":255,"wgCommentCodePointLimit":null,"wgWikiEditorMagicWords":{"redirect":"#REDIRECT","img_right":"right","img_left":"left","img_none":"none","img_center":"center","img_thumbnail":"thumb","img_framed":"frame","img_frameless":"frameless"},"mw.msg.wikieditor":"--~~~~"});var RLQ=window.RLQ||[];while(RLQ.length){RLQ.shift()();}window.RLQ={push:function(fn){fn();}};window.NORLQ={push:function(){}};}window.
mediaWikiLoadStart=mwNow();mwPerformance.mark('mwLoadStart');script=document.createElement('script');script.src="/load.php?debug=false&lang=en&modules=jquery%2Cmediawiki&only=scripts&skin=vector&version=0dyxpb6";script.onload=function(){script.onload=null;script=null;startUp();};document.head.appendChild(script);}());
