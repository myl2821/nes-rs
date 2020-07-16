MAP = {
  addr: 0..3,
  ins: 16..18,
  a: 50..51,
  x: 55..56,
  y: 60..61,
  p: 65..66,
  sp: 71..72,
  # ppu: 78..84,
  cyc: 90..-1
}

HEADER = %w(addr ins a x y p sp cyc)

puts HEADER.join ','
IO.foreach(ARGV[0]) do |line|
  line.rstrip!
  tokens = []
  MAP.each do |k, range|
    tokens << line[range]
  end
  puts tokens.join ','
end
