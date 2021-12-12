let { min, max, abs, floor, ceil, round, PI, cos, sin } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let gcd, lcm, atan, dist, diff, diffabs, cross, area, matran

function Rotor({
	N, // 转子顶角数
	E, // 偏心距
	P = (N + 0.5) * 0.4, // 转子顶半径 / 偏心距
	Q = P, // 转子腰半径 / 偏心距
	BP = 2, // 缸体转子间隙 / 顶半径 %
	tickn = 240, // 圆周步进数
	size, // 预估像素
}) {
	if ((N |= 0) < 2) throw 'err N'
	let NB = N - 1 // 缸体顶数
	let NBS = NB + NB // 缸体冲程数
	let NS = lcm(NBS, 4) // 完整循环冲程数
	let N2 = N + N
	tickn = ceil(tickn / N2 / NB) * N2 * NB // 圆周步进数，转子顶*缸体顶*2 的整倍数

	size = ceil(+size || min(size.width, size.height))
	E ??= round((size * 0.6875) / (N + P + 2.3)) / 2 // 偏心距
	let G = E * N // 转子大节圆半径
	let g = G - E // 曲轴小节圆半径
	P = round(E * (P + N + 2)) // 转子顶半径
	Q = round(E * (Q + N)) // 转子腰半径
	BP *= P / 100 // 缸体转子间隙

	// 转子、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = [...Array.seq(0, tickn)].map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tPQ = tickn / N2 // 转子顶腰步进
	let TPQ = PI2 / N2 // 转子顶腰夹角
	let tQ = n => (n * tickn) / N // 转子腰点起始步进
	let TQ = n => (n * PI2) / N // 转子腰点起始角
	let tS = s => ((s % NBS) * tickn) / NBS // 转子冲程起始步进
	let TS = s => ((s % NBS) * PI2) / NBS // 转子冲程起始角

	// 曲轴心 X=0 Y=0
	let GX = T => E * cos(T * N) // 转子心X
	let GY = T => E * sin(T * N) // 转子心Y
	let PX = (T, n = 0, p = P) => GX(T) + p * cos(T + TQ(n) + TPQ) // 转子顶X
	let PY = (T, n = 0, p = P) => GY(T) + p * sin(T + TQ(n) + TPQ) // 转子顶Y
	let QX = (T, n = 0, q = Q) => GX(T) + q * cos(T + TQ(n)) // 转子腰X
	let QY = (T, n = 0, q = Q) => GY(T) + q * sin(T + TQ(n)) // 转子腰X

	// 转子顶间腰线
	let PPt = [...Array.seq(-tPQ, tPQ, tickn)]

	// 缸体型线  E*cos(T*N-PI) + (P+BP)*cos(T), E*cos(T*N-PI) + (P+BP)*cos(T)
	let BB = Tick_.map(T => [PX(T - TPQ, 0, P + BP), PY(T - TPQ, 0, P + BP)])
	// 缸体步进角，非均匀
	let BT = BB.map(([X, Y]) => atan(X, Y))
	BT[BT.length - 1] = PI2
	// 缸体腰线、顶线步进
	let BQt = PPt
	let BPt = [...Array.seq(tS(1) - tPQ, tS(1) + tPQ)]
	// 缸体腰线、顶线步进角
	let BQT = BQt.map(BT.At())
	let BPT = BPt.map(BT.At())
	// 缸体冲程线步进
	let BSt = [...Array.seq(0, NBS - 1)].map(s => [
		...Array.seq(tS(s) - tPQ, tS(s + 1) + tPQ, tickn, true),
	])

	let TPB = (T, n = 0) => atan(PX(T, n), PY(T, n)) // 转子顶对应缸体角
	let tPB = (T, n = 0, int = round) => int(TPB(T, n).bfind(BT)) % tickn // 转子顶对应缸体步进
	// 转子顶与缸体接触角、及接触步进角
	let PBC = (T, n = 0) => {
		let CT = atan(PX(T, n) + g * cos(T * N), PY(T, n) + g * sin(T * N))
		return [CT - T - TQ(n) - TPQ, CT]
	}
	let PBCC = max(...Tick.map(T => PBC(T)[0])) // 最大接触角

	// 缸体对转子旋转
	function* RBT(B, TT) {
		if (typeof B == 'number')
			for (let T of TT) {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(T * (N - 1))
				let Y = P * sin(B + T) - E * sin(B * N + T) + E * sin(T * (N - 1))
				yield [atan(X, Y), dist(X, Y), X, Y] // 角[0,PI2) B==0 沿T严格递增 B>0 沿T循环严格递增
			}
		else for (let b of B) yield RBT(b, TT)
	}
	// 转子型线、即缸体绕转子心的内包络线
	let R = MinDot(RBT(BQt.map(Tick.At()), Tick), t => (t % tPQ ? null : t % (tickn / N) ? P : Q))
	// let R = MinCurve(RBT(Tick, Tick), true)

	// 工作容积，总容积，总体积
	let V, K, VV, KK, VB, KB
	{
		let v = area(BQt.map(BB.At())) - area(R(0, PPt))
		V = area(BPt.map(BB.At())) - area(R(TS(1), PPt)) - v
		VB = area(BB)
		VV = VB - area(R(0))
		;(K = V / v + 1), (KK = VV / V), (KB = VB / V)
	}

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { size, N, NS, E, G, g, P, Q, BP, V, K, VV, KK, VB, KB, PBCC })
	Object.assign(this, { TQ, TS, R })

	// 冲程区
	let SS = BSt.map((BSt, s) => {
		function* RR(T) {
			for (let [X, Y] of R(T, PPt)) yield [atan(X, Y), dist(X, Y), X, Y]
		}
		function* TT() {
			for (let t of Array.seq(tS(s), tS(s + 1), tickn, true)) yield RR(Tick_[t])
		}
		let rs = [...MinDot(TT(), null, BT, BSt)(0, BSt, 0, 0)]
		rs = BSt.map(BB.At()).concat(rs.reverse())
		return rs.push(rs[0]), rs
	})

	let params =
		_`N${N}__K${K}{}__E${E}{}__P${P}__` +
		_`V${VV / 100}{}/${V / 100}{}__=${KK}{1} VB${VB / 100}{}__` +
		_`BP${BP}{1} C${(PBCC / PI2) * 360}{}`
	console.log(...params.split('__'), `tn${tickn}`)

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		function $param(T = 0) {
			param.textContent =
				params.replace(/__/g, '\n') + _`|${(diffabs(PBC(T)[0]) / PI2) * 360}{02}`
		}
		let x = midx ?? canvas.width / 2 // 曲轴心X
		let y = midy ?? canvas.height / 2 // 曲轴心Y

		function $$({ color = '#000', opa = '', thick = 1 } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
		}
		// 画曲轴小圆
		function $g(style) {
			$$(style), $.arc(x, y, g, 0, PI2), $$$()
		}
		// 画偏心线
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo(x, y), $.lineTo(x + GX(T), y + GY(T)), $$$()
		}
		// 画转子大圆
		function $G(T, style) {
			$$({ color: '#999', ...style }), $.arc(x + GX(T), y + GY(T), G, 0, PI2), $$$()
		}
		// 画转子大圆凸包
		function $GG(style) {
			$$({ color: '#999', opa: '5', ...style }), $.arc(x, y, G + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $P(T, n = 0, O, style) {
			$$({ color: '#00f', ...style })
			BP && $.arc(x + PX(T, n), y + PY(T, n), BP, 0, PI2)
			$.moveTo(x + PX(T, n), y + PY(T, n)), $.lineTo(x + PX(T, n, O), y + PY(T, n, O))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			$$({ color: '#0f0', ...style })
			$.moveTo(x + QX(T, n), y + QY(T, n)), $.lineTo(x + QX(T, n, O), y + QY(T, n, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = G, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
			for (let T of Tick_) (T ? $.lineTo : $.moveTo).call($, x + QX(T), y + QY(T))
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画转子
		function $RR(T, pp, style) {
			$$(style)
			let to
			for (let [X, Y] of R(T, pp ? PPt : undefined))
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画转子接触角
		function $PBC(T, n = 0, O = P + BP + E, style) {
			$$({ color: '#66c', ...style })
			$.moveTo(x + PX(T, n, P + BP), y + PY(T, n, P + BP))
			$.lineTo(x + PX(T, n, O), y + PY(T, n, O))
			let CT = PBC(T, n)[1]
			$.moveTo(x + PX(T, n) + cos(CT), y + PY(T, n) + sin(CT))
			$.lineTo(x + PX(T, n) + (O - P) * cos(CT), y + PY(T, n) + (O - P) * sin(CT))
			$$$()
		}
		// 画冲程区
		function $SS(s, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SS[floor(s).mod(NBS)])
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		return Object.assign(
			{ param: $param, x, y, g: $g, Gg: $Gg, G: $G, GG: $GG },
			{ P: $P, Q: $Q, PN: $PN, QN: $QN, QQ: $QQ, BB: $BB, PBC: $PBC, RR: $RR, SS: $SS }
		)
	}

	// 点集求内包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function MinDot(dots, fix, TT = Tick_, tt = Tick_.keys()) {
		if (TT[0] != 0 || TT[tickn] != PI2) throw 'err TT'
		let M = new Array(tickn).fill(size * 2)
		for (let dot of dots)
			for (let [A, R] of dot) {
				let t = ceil(TT == Tick_ ? (tickn * A) / PI2 : A.bfind(TT)) % tickn
				M[t] = min(M[t], R)
			}
		tt.values ?? (tt = [...tt])
		// M.fillHole(size * 2) 如果tickn太小，部分步进无数据，则需要线性填充
		M[tt[0]] == size * 2 && (M[tt[0]] = M[(tt[0] + 1) % tickn])
		M[(tt.at(-1) + 1) % tickn] == size * 2 && (M[(tt.at(-1) + 1) % tickn] = M[tt.at(-1)])
		M[tickn] = M[0]
		for (let t of tt)
			if (t < tickn) M[t] = min(M[t], M[t + 1]) * 0.3125 + max(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) if ((d = fix(t, TT[t])) != null) M[t] = d
		M[tickn] = M[0]

		function* XY(T, ttt = tt, x = GX(T), y = GY(T)) {
			for (let t of ttt) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
		return (XY.count = tt.length - 1), XY
	}
}

gcd = function (a, b) {
	;(a |= 0), (b |= 0)
	for (let c; b; c = a % b, a = b, b = c);
	return a
}
lcm = (a, b) => ((a | 0) * (b | 0)) / gcd(a, b)
Number.prototype.mod = function (n) {
	return ((this % n) + n) % n
}

atan = (x, y) => (Math.atan(y / x) + (x <= 0 ? PI : PI2)) % PI2 // [0,PI2)
dist = (x, y) => Math.sqrt(x * x + y * y)
diff = v => (((v %= PI2) + v) % PI2) - v // [-PI,PI)
diffabs = v => abs(diff(v)) // [0,PI] ((v = abs(v) % PI2) > PI ? PI2 - v : v)

Array.prototype.At = function () {
	return this.at.bind(this)
}
// [from,to]序列
Array.seq = function* (from, to, wrap, oneAll) {
	if (wrap == null) for (; from <= to; from++) yield from
	else
		for (from = from.mod(wrap), to = to.mod(wrap); yield from, oneAll || from != to; )
			(from = (from + 1).mod(wrap)), (oneAll = false)
}
Number.prototype.bfind = function (s, prop, epsi) {
	let l = 0,
		h = s.length - 1
	for (let m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = this - (prop != null ? s[m][prop] : s[m]))
		if (epsi ? abs(c) <= EPSI : c == 0) return m
		c <= 0 ? (h = m - 1) : (l = m + 1)
	}
	let m = (prop != null ? s[h]?.[prop] + s[l]?.[prop] : s[h] + s[l]) / 2 // 超出为NaN
	return this < m ? h + 0.25 : l - 0.25
}

cross = function (ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
area = function (s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	return (a + (xx - x0) * (yy + y0)) / 2
}

Array.prototype.fillHole = function (hole) {
	let ii = this.findIndex(v => v != hole)
	if (ii < 0) throw 'all hole'
	let len = this.length
	let m = 0
	for (let j = ii, i = ii, k, n; (i = (i + 1) % len) != ii; j = i)
		if (this[i] == hole) {
			for (k = i; this[(k = (k + 1) % len)] == hole; );
			for (n = (k - j + len) % len, l = 1; l < n; l++, i = (i + 1) % len)
				this[i] = this[j] + ((this[k] - this[j]) * l) / n
			m += n - 1
		}
	return m
}
matran = function (row) {
	let col = []
	let i = 0
	for (let r of row) {
		let j = 0
		for (let c of r) (col[j++] ??= [])[i] = c
		i++
	}
	return col
}

// TODO 非闭合曲线不适用，需改进
if (false) {
	// 正向曲线集求内包络线 curves:[[ [A, R, X, Y] ]] tt:正向步进、可卷
	function MinCurve(curves, wrap, tt = Tick_.keys()) {
		let M
		for (let cc of curves) {
			cc.values ?? (cc = [...cc])
			if (!M) {
				M = cc // 初始化包络
				for (let moved, m = 1; m < M.length; m++)
					if (M[m - 1][0] > M[m - 1][0])
						if (moved || M[m - 1][0] < PI / 2 || M[m][0] > PI / 2) throw 'err curve A'
						else M.push(M.splice(0, m)), (moved = m = 1)
				continue
			}
			cc.forEach(c => (c[4] = c[0].bfind(M, 0)))
			let C = []
			for (let c of cc.keys()) {
				let [a, r, x, y, I0] = cc[c++]
				if (!cc[c] && !wrap) continue
				let [a_, r_, x_, y_, I9] = cc[c] ?? cc[0]
				;(I0 = floor(I0)), (I9 = ceil(I9)) // 遍历曲线连续段，对应包络连续段
				let IC = I0 + 1
				if (I0 < 0) I0 = wrap ? M.length - 1 : 0
				if (I9 == M.length) I9 = wrap ? 0 : M.length - 1
				for (let I = I0, I1, I_; I != I9; I = I_) {
					let [A, R, X, Y] = M[I]
					let [A_, R_, X_, Y_] = M[(I_ = I1 = I + 1)] ?? M[(I_ = 0)]
					// 求交点和半径
					let [SX, SY, ABc, ABd, cdA, cdB] = cross(X, Y, X_, Y_, x, y, x_, y_)
					if (I_ != I9 && cdB < -EPSI && I_) C.push([0 / 0, 0, 0, 0, I_]) // 去掉长半径包络点
					if (I == I0 && ABc > EPSI) C.push([a, r, x, y, IC]) // 短半径曲线点
					if (I_ == I9 && ABd > EPSI) C.push([a_, r_, x_, y_, I1]) // 短半径曲线点
					if (SX != null) C.push([atan(SX, SY), dist(SX, SY), SX, SY, I1]) //交点
				}
			}
			C.sort((c, d) => c[4] - d[4] || c[0] - d[0]) // 下标和角度顺序
			for (let c = 1; c < C.length; c++) if (M[m][0] < M[m - 1][0]) throw 'err curve A'
			let i = 0 // 加减点
			for (let c of C)
				if (c[0] != c[0]) M.splice(c[4] + i--, 1)
				else M.splice(c[4] + i++, 0, c), c.length--
			M.push((([A, ...M]) => [A + PI2, ...M])(M[0])) // 闭合
			i = 0 // 合并重合点
			for (let m of M)
				if (m[0] < M[i][0]) throw 'err a'
				else if (m[0] - M[i][0] > EPSI && abs(m[1] - M[i][1]) > EPSI) M[++i] = m
			M.length = ++i
		}
		tt.values ?? (tt = [...tt])
		function* XY(T, ttt = tt, x = GX(T), y = GY(T)) {
			ttt = ttt.values?.() ?? ttt
			let t = ttt.next().value
			if (t == null) return
			let m = round(Tick_[t].bfind(M, 0))
			for (let t_ of ttt)
				do
					for (let A, T_ = Tick_[t_] + (t > t_ && PI2); (A = M[m]?.[0]) <= T_; m++)
						yield [x + M[m][1] * cos(T + A), y + M[m][1] * sin(T + A)]
				while (t > (t = t_) && !(m = 0))
		}
		return (XY.count = M.length - 1), XY
	}
}
